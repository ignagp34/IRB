"""Interactive natural-language chat for the IRB-120 cognitive cell.

Thin client of the reasoning node's ``ExecuteInstruction`` service: every line
the user types is forwarded to the LLM tool-using agent, so the arm can be
driven in plain language without hand-writing ``ros2 service call`` commands.

It adds no robot-control logic of its own and respects the module boundaries —
all reasoning, validation and motion still happen behind the existing service.
"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node

from irb120pe_cognitive_interfaces.srv import ExecuteInstruction


EXIT_WORDS = {"salir", "exit", "quit", "q", "adios", "adiós", "bye"}

BANNER = """
============================================================
  IRB-120 — Chat en lenguaje natural
============================================================
  Escribe una orden y pulsa Enter. Ejemplos:
    - ordena los cubos por color de blanco a negro a azul
    - coge el cubo azul y ponlo en el contenedor de la izquierda
    - haz una torre con los cubos
  Para salir: escribe 'salir' (o pulsa Ctrl+D).
------------------------------------------------------------
  Consejo: lanza la demo con llm_provider:=openrouter para
  entender lenguaje libre. En 'mock' usa palabras clave.
============================================================
"""


class ChatClient(Node):
    def __init__(self) -> None:
        super().__init__("irb120pe_chat_client")
        self.declare_parameter("execute_instruction_service", "/irb120pe/reasoning/execute_instruction")
        self.declare_parameter("show_trace", True)
        self.service_name = str(self.get_parameter("execute_instruction_service").value)
        self.show_trace = bool(self.get_parameter("show_trace").value)
        self.client = self.create_client(ExecuteInstruction, self.service_name)

    def wait_ready(self, timeout_sec: float = 10.0) -> bool:
        return self.client.wait_for_service(timeout_sec=timeout_sec)

    def ask(self, instruction: str) -> ExecuteInstruction.Response | None:
        request = ExecuteInstruction.Request()
        request.instruction = instruction
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def _format_trace(tool_trace: str) -> str | None:
    if not tool_trace or tool_trace in ("[]", ""):
        return None
    try:
        items = json.loads(tool_trace)
    except json.JSONDecodeError:
        return None
    if not items:
        return None
    lines: list[str] = []
    for item in items:
        if not isinstance(item, dict):
            lines.append(f"   - {item}")
            continue
        if "tool" in item:
            label = item.get("tool")
            result = item.get("result", item.get("output", ""))
            lines.append(f"   - {label}: {result}")
        elif "agent_output" in item:
            provider = item.get("provider", "llm")
            lines.append(f"   - {provider}: {item['agent_output']}")
        else:
            lines.append(f"   - {json.dumps(item, ensure_ascii=False)}")
    return "\n".join(lines)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChatClient()
    print(BANNER)
    if not node.wait_ready(10.0):
        print(f"No encuentro el servicio '{node.service_name}'.")
        print("¿Lanzaste la demo? (ros2 launch irb120pe_cognitive cognitive_arrangement_demo.launch.py ...)")
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        while True:
            try:
                instruction = input("\n🤖 > ").strip()
            except EOFError:
                break
            if not instruction:
                continue
            if instruction.lower() in EXIT_WORDS:
                break

            print("   ...pensando y ejecutando (puede tardar mientras el brazo se mueve)...")
            try:
                response = node.ask(instruction)
            except KeyboardInterrupt:
                print("\n(orden interrumpida)")
                continue

            if response is None:
                print("⚠️  No hubo respuesta del razonador.")
                continue

            mark = "✅" if response.success else "❌"
            print(f"{mark} {response.status}")
            if node.show_trace:
                trace = _format_trace(response.tool_trace)
                if trace:
                    print("   herramientas usadas:")
                    print(trace)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n👋 Chat cerrado.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

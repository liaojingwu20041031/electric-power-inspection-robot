import json
from typing import Any, Dict

from std_msgs.msg import String

from .agent_schema import tool_result


SYSTEM_TOOLS = {
    'start_patrol_mode',
    'pause_patrol',
    'resume_patrol',
    'cancel_patrol',
    'emergency_stop',
}


class AgentTools:
    def __init__(self, node, state, system_pub, motion_pub, say_pub, event_pub) -> None:
        self.node = node
        self.state = state
        self.system_pub = system_pub
        self.motion_pub = motion_pub
        self.say_pub = say_pub
        self.event_pub = event_pub

    def execute(self, decision: Dict[str, Any], policy) -> Dict[str, Any]:
        tool_call = decision['tool_call']
        name = str(tool_call['name'])
        args = tool_call.get('arguments') or {}
        if not policy.allowed:
            result = tool_result(name, False, 'rejected', policy.reason, error_code='policy_rejected')
            self.publish_event(result)
            return result

        if name in SYSTEM_TOOLS:
            payload = {'schema_version': '1.0', 'source': 'inspection_agent'}
            payload.update({key: value for key, value in args.items() if key != 'command'})
            payload['command'] = name
            self.publish_json(self.system_pub, payload)
            result = tool_result(name, True, 'sent', f'已发送系统命令: {name}', {'command': name})
        elif name == 'send_motion_command':
            command = str(args.get('command') or '')
            self.publish_json(self.motion_pub, {
                'schema_version': '1.0',
                'source': 'inspection_agent',
                'command': command,
                'request_id': str(decision.get('decision_id') or ''),
                'timestamp': self.node.get_clock().now().nanoseconds / 1e9,
            })
            result = tool_result(name, True, 'sent', '已发送运动命令', {'command': command})
        elif name == 'get_system_status':
            result = tool_result(name, True, 'ok', 'system status', self.state.system_status)
        elif name == 'get_patrol_status':
            result = tool_result(name, True, 'ok', 'patrol status', self.state.patrol_status)
        elif name == 'get_voice_status':
            result = tool_result(name, True, 'ok', 'voice status', self.state.voice_status)
        else:
            speak = decision.get('speak') or {}
            answer = str(decision.get('final_answer') or speak.get('text') or '当前状态未知。')
            result = tool_result(name, True, 'ok', answer, {'answer': answer})

        self.publish_event(result)
        return result

    def say(self, decision: Dict[str, Any], priority: int = 5, interrupt: bool = False) -> None:
        speak = decision.get('speak') or {}
        text = str(speak.get('text') or decision.get('final_answer') or '')
        if not text:
            return
        from ylhb_interfaces.msg import SayText

        msg = SayText()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.task_id = str(decision.get('decision_id') or 'inspection_agent')
        msg.priority = int(speak.get('priority') or priority)
        msg.interrupt = bool(speak.get('interrupt') or interrupt)
        msg.text = text
        self.say_pub.publish(msg)

    def publish_event(self, payload: Dict[str, Any]) -> None:
        self.publish_json(self.event_pub, payload)

    @staticmethod
    def publish_json(pub, payload: Dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        pub.publish(msg)

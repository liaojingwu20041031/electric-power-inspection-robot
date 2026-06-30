BAD_TONE_WORDS = ('亲', '哦', '我觉得吧')

REPLIES = {
    'emergency_stop': ('收到，已急停。', 10, True),
    'motion_stop': ('已停止短时运动。', 5, False),
    'patrol_start': ('收到，开始巡逻。', 5, False),
    'patrol_pause': ('收到，暂停巡逻。', 5, False),
    'patrol_resume': ('收到，继续巡逻。', 5, False),
    'patrol_cancel': ('收到，取消巡逻。', 5, False),
    'capability_query': ('我是电力巡检机器人，可以执行巡逻控制、短时运动和状态查询。', 5, False),
    'debug_query': ('我已收到问题。请检查麦克风、网络和日志状态。', 5, False),
    'unknown_local': ('我还不能确定这条指令，请换一种说法，或查看语音和系统状态。', 5, False),
}


def speak(intent: str, text: str = '') -> dict:
    reply, priority, interrupt = REPLIES.get(intent, (text, 5, False))
    return {
        'reply_key': f'command.{intent}',
        'text': sanitize_reply(text or reply),
        'priority': priority,
        'interrupt': interrupt,
    }


def sanitize_reply(text: str) -> str:
    cleaned = str(text)
    for word in BAD_TONE_WORDS:
        cleaned = cleaned.replace(word, '')
    return cleaned.strip()

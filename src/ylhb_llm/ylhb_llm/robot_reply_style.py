import re


BAD_TONE_WORDS = ('亲', '哦', '我觉得吧')
DETAIL_SUFFIX = '详细内容请看屏幕。'

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


def prepare_speech_text(text: str, max_chars: int = 60, max_sentences: int = 2) -> str:
    original = str(text or '').strip()
    cleaned = re.sub(r'```[\s\S]*?```', ' ', original)
    cleaned = re.sub(r'!\[[^]]*]\([^)]*\)', ' ', cleaned)
    cleaned = re.sub(r'\[([^]]+)]\([^)]*\)', r'\1', cleaned)
    cleaned = re.sub(r'https?://\S+|www\.\S+', ' ', cleaned)
    cleaned = re.sub(
        r'<[^>]+>|\b(?:tool_name|tool_call_id|assistant_chat_[\w-]+|[a-z][a-z0-9]*(?:_[a-z0-9]+)+)\b',
        ' ', cleaned)
    cleaned = re.sub(r'[`#>*_~|]+', ' ', cleaned)
    cleaned = re.sub(r'\s+', ' ', cleaned).strip()
    if not cleaned:
        return '详细内容请看屏幕。'

    sentences = [item.strip() for item in re.findall(r'[^。！？]+[。！？]?', cleaned) if item.strip()]
    sentences = [item if item[-1] in '。！？' else item + '。' for item in sentences]
    complete = ''.join(sentences)
    sentence_limit = max(1, int(max_sentences))
    if len(sentences) <= sentence_limit and len(complete) <= max_chars:
        return complete
    selected = ''
    body_limit = max(0, max_chars - len(DETAIL_SUFFIX))
    for sentence in sentences[:max(0, sentence_limit - 1)]:
        if len(selected + sentence) > body_limit:
            break
        selected += sentence
    if selected:
        return selected + DETAIL_SUFFIX
    return '结果较长，详细内容请看屏幕。'

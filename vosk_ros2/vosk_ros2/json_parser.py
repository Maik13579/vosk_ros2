import json
from vosk_ros2_interfaces.msg import RecognizedSpeech, Result, Alternative

def parse_vosk_json(json_str: str) -> RecognizedSpeech:
    data = json.loads(json_str)

    msg = RecognizedSpeech()
    msg.text = data.get("text", "")
    msg.spk = data.get("spk", 0)

    # Parse word-level results
    if "result" in data:
        for r in data["result"]:
            res = Result()
            res.conf = float(r.get("conf", 0.0))
            res.start = float(r.get("start", 0.0))
            res.end = float(r.get("end", 0.0))
            res.word = r.get("word", "")
            msg.results.append(res)

    # Parse alternatives
    if "alternatives" in data:
        for alt in data["alternatives"]:
            a = Alternative()
            a.text = alt.get("text", "")
            a.confidence = float(alt.get("confidence", 0.0))
            msg.alternatives.append(a)

    return msg


# Usage Example
if __name__ == "__main__":
    sample_json = """
    {
      "text": "hello world",
      "spk": 1,
      "result": [
        { "conf": 0.96, "start": 0.70, "end": 1.10, "word": "hello" },
        { "conf": 0.88, "start": 1.10, "end": 1.50, "word": "world" }
      ],
      "alternatives": [
        { "text": "hello world", "confidence": 0.85 },
        { "text": "yellow world", "confidence": 0.15 }
      ]
    }
    """
    recognized_speech_msg = parse_vosk_json(sample_json)
    print("Recognized Text:", recognized_speech_msg.text)
    print("Speaker ID:", recognized_speech_msg.spk)
    for i, res in enumerate(recognized_speech_msg.results):
        print(f" Word {i}: {res.word}, conf={res.conf}")
    for i, alt in enumerate(recognized_speech_msg.alternatives):
        print(f" Alternative {i}: {alt.text}, conf={alt.confidence}")

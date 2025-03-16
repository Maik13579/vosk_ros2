import json
from vosk_ros2_interfaces.msg import RecognizedSpeech, Result, Alternative

def parse_vosk_json(json_str: str) -> RecognizedSpeech:
    data = json.loads(json_str)

    msg = RecognizedSpeech()
    msg.text = data.get("text", "")
    # Our JSON returns a speaker embedding in "spk"
    # We'll store that in spk_embedding and leave spk (speaker ID) empty.
    msg.spk = "" #TODO
    msg.spk_embedding = data.get("spk", [])
    msg.spk_frames = data.get("spk_frames", 0)

    # Compute elapsed time from the "result" array if available.
    results = data.get("result", [])
    if results:
        first_start = float(results[0].get("start", 0.0))
        last_end = float(results[-1].get("end", 0.0))
        msg.elapsed_time = last_end - first_start
    else:
        msg.elapsed_time = 0.0

    # Parse word-level results.
    for r in results:
        res = Result()
        res.conf = float(r.get("conf", 0.0))
        res.start = float(r.get("start", 0.0))
        res.end = float(r.get("end", 0.0))
        res.word = r.get("word", "")
        msg.results.append(res)

    # Parse alternatives.
    if "alternatives" in data:
        for alt in data["alternatives"]:
            a = Alternative()
            a.text = alt.get("text", "")
            a.confidence = float(alt.get("confidence", 0.0))
            if "result" in alt:
                for w in alt["result"]:
                    res = Result()
                    #res.conf = float(w.get("conf", 0.0)) #Not set in alternatives
                    res.start = float(w.get("start", 0.0))
                    res.end = float(w.get("end", 0.0))
                    res.word = w.get("word", "")
                    a.results.append(res)
            msg.alternatives.append(a)

    return msg


# Usage Example
if __name__ == "__main__":
    sample_json = """
    {
      "result": [
        { "conf": 1.0, "end": 0.66, "start": 0.0, "word": "hello" },
        { "conf": 1.0, "end": 1.14, "start": 0.66, "word": "world" }
      ],
      "spk": [
        -0.697553, -0.919185, -0.303348, -0.396414, -0.433837, -0.884268,
        1.492474, 0.495517, 0.393541, 0.819232, 0.460984, 1.49378,
        -1.228134, -0.528948, 1.26093, 2.722531, 1.299404, 1.323298,
        -0.846605, 0.162527, 0.665113, 1.091728, -0.459468, 1.225677,
        1.358261, 0.533101, 0.662442, -0.22702, -0.950497, 0.643873,
        0.742184, 0.489364, 0.418284, -0.31449, -1.393756, 0.762709,
        1.106673, -1.514205, 0.150269, 0.326629, 0.371568, -0.404355,
        0.326484, 0.760649, 0.19813, -0.157739, -0.43601, -1.420456,
        1.856828, -0.156054, 1.865741, -0.917024, -0.716493, -0.435284,
        0.639535, -0.176695, -1.415807, -0.185, -0.403578, -0.758717,
        0.247278, 0.942784, -2.111673, 0.356831, -1.662069, -0.659634,
        -1.128506, -0.441024, -0.459756, -1.326232, 1.254191, 1.561751,
        -0.139089, 2.504374, -0.309438, 1.154654, -1.083965, -0.168301,
        0.496053, 0.967727, -1.420024, -0.288265, -0.34077, -1.159668,
        1.478663, 1.256337, -2.146639, -0.889864, -1.915701, -0.094786,
        -0.697037, 0.394071, 0.104928, 0.921164, 0.298725, 0.931527,
        0.999871, -1.321667, 0.60292, 1.339599, -0.363243, 0.174857,
        1.045289, -0.388484, 2.668365, -0.511324, -1.333479, -0.042952,
        0.478149, -1.762667, 0.817189, -1.176626, 0.064365, 0.938362,
        -0.766349, -1.245966, 1.201638, -1.238893, -0.005755, -0.040019,
        -0.215374, 0.678462, 0.363382, -0.144798, 1.391493, 0.424203,
        -0.654633, -0.134743
      ],
      "spk_frames": 111,
      "text": "hello world"
    }
    """
    recognized_speech_msg = parse_vosk_json(sample_json)
    print("Recognized Text:", recognized_speech_msg.text)
    print("Speaker Embedding:", recognized_speech_msg.spk_embedding)
    print("Speaker Frames:", recognized_speech_msg.spk_frames)
    for i, res in enumerate(recognized_speech_msg.results):
        print(f"Word {i}: {res.word}, conf={res.conf}, start={res.start}, end={res.end}")
    for i, alt in enumerate(recognized_speech_msg.alternatives):
        print(f"Alternative {i}: {alt.text}, conf={alt.confidence}")
        for j, w in enumerate(alt.results):
            print(f"  Word {j}: {w.word}, conf={w.conf}, start={w.start}, end={w.end}")

# Vosk ROS2

A ROS 2 package that provides a ROS 2 interface for the [Vosk](https://github.com/alphacep/vosk-api) speech recognition engine.  
It supports grammar‐restricted recognition, speaker recognition, and a simple speaker database.  
The node exposes a SpeechDetection action server and two services: SetGrammar and AddSpeaker.

## Overview

- **SpeechDetection Action:**  
  - **Goal:**  
    - `continuous` (bool): If true, the node continuously listens for speech until canceled.
    - `publish_partial` (bool): If true, partial results are sent as feedback.
  - **Result:**  
    - `final_result`: A `RecognizedSpeech` message containing the final recognized text, word‐level results, speaker data (if available), and elapsed time.
    - `success` (bool): Indicates whether recognition succeeded.
    - `message` (string): Provides error or cancellation messages.
  - **Feedback:**  
    - `continuous_result`: A `RecognizedSpeech` message with the latest recognition (for continuous mode).
    - `partial_text` (string): The current partial recognition text.

- **SetGrammar Service:**  
  Update the grammar used by Vosk at runtime.  
  *Request:*  
  - `path`: (string) Absolute path to a grammar file.  
  - `grammar`: (string array) List of grammar lines (if no file is provided).  
  - `return_expansions` (bool): If true, returns all expanded sentences.  
  *Response:*  
  - `expansions`: (string array) The list of expanded sentences.
  - `success` (bool): True if the grammar was set successfully.
  - `message` (string): Error message if something went wrong.

- **AddSpeaker Service:**  
  Enroll a new speaker in a simple in-memory (and file-backed) speaker database.  
  *Request:*  
  - `id` (string): Unique speaker identifier.
  - `embedding` (float32 array): Speaker embedding (if empty, the last recognized embedding is used).  
  *Response:*  
  - `success` (bool)
  - `message` (string)

- **Speaker Recognition:**  
  After recognition, if the `RecognizedSpeech` result contains a non-empty `spk_embedding`, the node compares it with a stored database of speaker embeddings using cosine distance.  
  The result message’s `spker` field is filled with a list of `Speaker` messages (each containing a unique ID and cosine distance), sorted in order of increasing distance (most similar first).

## Grammar
This grammar language is designed to create a set of predefined sentences by using a simple syntax. It supports the following features:

- **Alternatives (`|`):**  
  Use the vertical bar to separate alternative tokens or phrases.  
  _Example:_  hello | hi

This means either "hello" or "hi" can be chosen.

- **Grouping (`( ... )`):**  
Parentheses group tokens together, which is useful for combining alternatives or sequences.  
_Example:_  good (morning | evening)

This expands to "good morning" and "good evening".

- **Optional Elements (`[ ... ]`):**  
Square brackets denote that a token or group is optional.  
_Example:_  hello [there]

This expands to both "hello" and "hello there".

- **References (`<tag>`):**  
You can define a reference with a line like:  
`&lt;day&gt;`: monday | tuesday | wednesday

Then use `<day>` in other grammar lines to substitute the defined alternatives.  
*Note:* The parser converts all tokens to lower case.

- **Lower-case Enforcement:**  
All tokens are converted to lower-case to ensure consistency.

- **No Repetition Operators:**  
The grammar does not support operators like `+` or `*` to prevent infinite expansions.

The parser processes reference definitions first and then expands the normal grammar lines by replacing references on-demand. If there is an undefined reference or a cyclic dependency, it raises a `GrammarParseError`.

## How It Works
   
1. **Grammar:**  
   - You can set a grammar either by loading a file from the package’s share directory (e.g., `grammar/ask_name`) or by calling the SetGrammar service.
   - The grammar is expanded (using a custom parser) into a strict list of complete sentences that Vosk uses to constrain recognition.

2. **Speaker Database:**  
   - A simple database (encapsulated in the `SpeakerDatabase` class) stores speaker embeddings.
   - When a recognition result is produced with a speaker embedding, it is compared against the database using cosine distance.
   - The sorted results are added to the result message’s `spker` field.

## Parameters

- **model_path**:  
  The file system path to the Vosk speech model directory.  
  _Example:_ `/opt/vosk_model/speech_model`

- **speaker_model_path**:  
  The path to the speaker model used for speaker recognition.  
  _Example:_ `/opt/vosk_model/speaker_model`

- **sample_rate**:  
  The audio sample rate in Hertz used for capturing and processing audio.  
  _Example:_ `48000.0`

- **block_size**:  
  The number of audio frames processed per block. A larger block size may reduce CPU load but can increase latency.  
  _Example:_ `16000`

- **silence_timeout**:  
  The time in seconds to wait without receiving any partial updates before stopping recognition.
  This is only used when nobody speaks at all, vosk will stop by its own as soon as something was said.
  _Example:_ `5.0`

- **max_alternatives**:  
  The maximum number of alternative recognition hypotheses to return. Setting this to `0` (or a value less than 1) disables alternatives, which in turn enables speaker recognition. (For some reason you can't have alternatives and speaker recognition at the same time)
  _Example:_ `0` (disabled)

- **set_words**:  
  Set the recognizer to return word-level results.
  _Example:_ `True`

- **grammar_file**:  
  The name (or relative path) of the file containing grammar definitions. If not an absolute path, it is assumed to be located in the package’s grammar directory.  
  _Example:_ `ask_name`

- **db_file**:  
  The file path for storing the speaker database. This file is used to save and load speaker embeddings.  
  If empty, no persistence is used.
  _Example:_ `/speakers/speakers.json`


## Adaptation
The grammar is only supported for models with dynamic graph (see https://alphacephei.com/vosk/adaptation). Furthermore, the grammar is only updating the vocabulary of the model. So if your grammar is ["one", "two three", "four"] the model can still return "two four".

In order to set a precisely defined grammar, you can use the `scripts/build_fst.sh` script. This script converts a [JSGF](https://www.w3.org/TR/jsgf/) file into an FSM and compiles it into an FST. The models that you can download from https://alphacephei.com/vosk/models already come with an Grammar (`Gr.fst` in the `graph` folder). With this script you can create a new model with a custom grammar.

***IMPORTANT***: You cannot add new words to the model using the `scripts/build_fst.sh` script. The JSGF file is only allowed to contain words that are already in the model (typically the `graph/words.txt` file). And only lowercase letters are allowed (and no punctuation).

I tested it with this model `vosk-model-en-us-0.22-lgraph`, don't know if the script works for other models.

If you need to add new words to the vosk model, check out this page https://alphacephei.com/vosk/lm.

An example of JSGF files can be found in the [scripts](./scripts) folder.
- demo.jsgf is a small example stolen from https://github.com/alphacep/vosk-api/issues/55
- gpsr.jsgf is a bigger example for the Robocup@Home [GPSR Command Generator](https://github.com/RoboCupAtHome/CommandGenerator)

## Installation

1. **Dependencies:**  
   - ROS 2 (Humble or later)  
   - Python 3  
   - Vosk (install with `pip3 install vosk`)  
   - sounddevice (`pip3 install sounddevice`)  
   - NumPy (`pip3 install numpy`)  
   - ament_index_python (comes with ROS 2)

2. **Build the Package:**  
   Place the package in your ROS 2 workspace under `src` and build:
   ```bash
   colcon build --symlink-install


Or just do
```bash
docker compose -f docker/compose.yaml up
```
#!/usr/bin/env python3

import sys
import json
import queue
import time
import sounddevice as sd
import vosk


class VoskSpeechRecognizer:
    """
    Vosk-based speech recognizer with optional grammar constraints and speaker recognition.
    
    1) Grammar:
       - Expects a list of plain strings. If empty => free-form recognition.
    2) Speaker Recognition:
       - Provide speaker_model_path to get speaker embeddings. We call SetSpkModel() after creation.
    3) Microphone capture:
       - run_mic() uses sounddevice to capture audio in real time.
    """

    def __init__(self,
                 model_path: str,
                 sample_rate: float = 16000.0,
                 speaker_model_path: str = None,
                 MAX_ALTERNATIVES: int = None):
        """
        :param model_path: Path to the Vosk model directory (e.g. "/opt/vosk_model/speech_model")
        :param sample_rate: Audio sample rate (e.g. 16000)
        :param speaker_model_path: Path to speaker model (e.g. "/opt/vosk_model/speaker_model"), if any
        :param MAX_ALTERNATIVES: Maximum number of alternatives to return, disables speaker recognition
        """
        # Load main Vosk model
        self.model = vosk.Model(model_path)
        self.MAX_ALTERNATIVES = MAX_ALTERNATIVES

        # Optionally load speaker model
        self.spk_model = None
        if speaker_model_path:
            self.spk_model = vosk.SpkModel(speaker_model_path)

            if self.MAX_ALTERNATIVES is not None:
                print("Warning: Speaker model is ignored because MAX_ALTERNATIVES is set.")

        self.sample_rate = sample_rate

        # Create a default (free-form) recognizer
        self.set_grammar([])

        # Internal queue
        self.audio_queue = queue.Queue()
        self.running = False
        self.tts_status = False # Set to true to disable listening (that robot dont listen to itself)

    def set_grammar(self, sentence_list):
        """
        Re-initializes the Vosk recognizer with a plain list of sentences.
        If the list is empty, revert to free-form recognition.
        Then re-attach the speaker model if we have one.
        """
        if not sentence_list:
            self.recognizer = vosk.KaldiRecognizer(self.model, self.sample_rate)
        else:
            grammar_json = json.dumps(sentence_list)
            self.recognizer = vosk.KaldiRecognizer(self.model, self.sample_rate, grammar_json)

        self.recognizer.SetWords(True)
        if self.MAX_ALTERNATIVES:
            self.recognizer.SetMaxAlternatives(self.MAX_ALTERNATIVES)

        if self.spk_model:
            self.recognizer.SetSpkModel(self.spk_model)


    def get_partial_result(self) -> str:
        """
        Returns the latest partial result as a JSON string, e.g.:
          {"partial": "hello wor"}
        """
        return self.recognizer.PartialResult()

    def get_final_result(self) -> str:
        """
        Returns the final result as a JSON string, which may contain "spk" or "speaker".
        Example:
        {
          "text": "hello world",
          "spk": [ ... ],
          "result": [...],
          "alternatives": [...]
        }
        """
        return self.recognizer.FinalResult()

    def run_mic(self, block_size=8000, silence_timeout=3.0):
        """
        Captures audio from the default microphone using sounddevice.
        If no new partial result is detected for 'silence_timeout' seconds,
        the capture stops.
        
        :param block_size: Number of frames per read from sounddevice.
        :param silence_timeout: Time in seconds to wait for a change in partial result before stopping.
        """
        try:
            with sd.RawInputStream(
                samplerate=self.sample_rate,
                blocksize=block_size,
                dtype='int16',
                channels=1,
                callback=self._sd_callback
            ):
                self.running = True
                last_non_silence_time = time.time()
                last_partial = ""
                while self.running:

                    if self.tts_status: #Clear the queue
                        with self.audio_queue.mutex:
                            self.audio_queue.queue.clear()
                        self.recognizer.Reset()
                        continue

                    # Get data from the queue and process it
                    data = self.audio_queue.get()
                    if self.recognizer.AcceptWaveform(data):
                        self.running = False
                        break
    
                    pr = self.get_partial_result()
                    if '"partial"' in pr:
                        partial_data = json.loads(pr)
                        text = partial_data.get("partial", "")
                        # Update timer only if partial result changed and is non-empty.
                        if text and text != last_partial:
                            last_partial = text
                            last_non_silence_time = time.time()
                    # Stop if silence detected for too long.
                    if time.time() - last_non_silence_time > silence_timeout:
                        print("Silence detected, stopping recognition.")
                        self.running = False
                        break
        except KeyboardInterrupt:
            print("\nInterrupted, stopping microphone capture...")
            self.running = False
        except Exception as e:
            print(f"[Error] {e}", file=sys.stderr)
            self.running = False

        
    def _sd_callback(self, indata, frames, time_info, status):
        """
        Sounddevice callback.
        """
        if status:
            print(f"[Audio Status] {status}", file=sys.stderr)
        if self.running:
            data_bytes = bytes(indata)
            self.audio_queue.put(data_bytes)


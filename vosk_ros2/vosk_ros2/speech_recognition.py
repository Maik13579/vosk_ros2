#!/usr/bin/env python3

import sys
import json
import queue
import threading
import sounddevice as sd
import vosk


class VoskSpeechRecognizer:
    """
    Vosk-based speech recognizer with optional grammar constraints and speaker recognition.
    
    1) Grammar:
       - Expects a list of plain strings. If empty => free-form recognition.
    2) Speaker Recognition:
       - Provide speaker_model_path to get speaker embeddings. We call .SetSpkModel() after creation.
    3) Microphone capture:
       - run_mic() uses sounddevice to capture audio in real time.
    """

    def __init__(self,
                 model_path: str,
                 sample_rate: float = 16000.0,
                 speaker_model_path: str = None):
        """
        :param model_path: Path to the Vosk model directory (e.g. "/opt/vosk_model/speech_model")
        :param sample_rate: Audio sample rate (e.g. 16000)
        :param speaker_model_path: Path to speaker model (e.g. "/opt/vosk_model/speaker_model"), if any
        """
        # Load main Vosk model
        self.model = vosk.Model(model_path)

        # Optionally load speaker model
        self.spk_model = None
        if speaker_model_path:
            self.spk_model = vosk.SpkModel(speaker_model_path)

        self.sample_rate = sample_rate

        # Create a default (free-form) recognizer
        self.recognizer = vosk.KaldiRecognizer(self.model, self.sample_rate)

        # If we have a speaker model, attach it
        if self.spk_model:
            self.recognizer.SetSpkModel(self.spk_model)

        # Internal queue for optional background processing
        self.audio_queue = queue.Queue()
        self.running = False
        self.thread = None

    def set_grammar(self, sentence_list):
        """
        Re-initializes the Vosk recognizer with a plain list of sentences.
        If the list is empty, revert to free-form recognition.

        Then re-attach the speaker model if we have one.
        """
        if not sentence_list:
            # Free-form
            self.recognizer = vosk.KaldiRecognizer(self.model, self.sample_rate)
        else:
            grammar_json = json.dumps(sentence_list)
            self.recognizer = vosk.KaldiRecognizer(self.model, self.sample_rate, grammar_json)

        if self.spk_model:
            self.recognizer.SetSpkModel(self.spk_model)

    def start_recognition(self):
        """
        Starts a background thread to consume queued audio data (if using add_audio_chunk()).
        """
        self.running = True
        self.thread = threading.Thread(target=self._process_audio_queue, daemon=True)
        self.thread.start()

    def stop_recognition(self):
        """
        Stops processing and joins the background thread.
        """
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join()

    def add_audio_chunk(self, chunk: bytes):
        """
        Queues raw audio data (16-bit PCM) for recognition if using the background thread approach.
        """
        self.audio_queue.put(chunk)

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
          "spk": [0.123, -0.456, ...],
          "result": [...],
          ...
        }
        """
        return self.recognizer.FinalResult()

    def run_mic(self, block_size=8000, show_partial=True):
        """
        Captures audio from the default microphone using sounddevice,
        processes it in real-time, and prints partial/final results.
        :param block_size: Number of frames per read from sounddevice
        :param show_partial: If True, prints partial results as recognized
        """
        try:
            with sd.RawInputStream(
                samplerate=self.sample_rate,
                blocksize=block_size,
                dtype='int16',
                channels=1,
                callback=self._sd_callback
            ):
                print(f"Listening at {self.sample_rate} Hz. Press Ctrl+C to stop.")
                # Keep running until user interrupts or external code sets self.running=False
                self.running = True
                while self.running:
                    sd.sleep(100)
                    if show_partial:
                        pr = self.get_partial_result()
                        if '"partial"' in pr:
                            partial_data = json.loads(pr)
                            text = partial_data.get("partial", "")
                            if text:
                                print(f"Partial: {text}", flush=True)
        except KeyboardInterrupt:
            print("\nInterrupted, stopping microphone capture...")
        except Exception as e:
            print(f"[Error] {e}", file=sys.stderr)

        # Print final result
        final_json = self.get_final_result()
        if final_json:
            final_data = json.loads(final_json)
            print("\nFinal text:", final_data.get("text", ""))
            # If we have a speaker embedding or ID:
            if "spk" in final_data:
                print("Speaker embedding:", final_data["spk"])
            elif "speaker" in final_data:
                print("Speaker ID:", final_data["speaker"])

    def _sd_callback(self, indata, frames, time_info, status):
        """
        Sounddevice callback. Converts the incoming cffi buffer to bytes
        and feeds it to Vosk for immediate recognition.
        """
        if status:
            print(f"[Audio Status] {status}", file=sys.stderr)
        if self.running:
            # Convert the incoming buffer to bytes.
            data_bytes = bytes(indata)
            self.recognizer.AcceptWaveform(data_bytes)

    def _process_audio_queue(self):
        """
        Thread function to continuously read from self.audio_queue and feed the recognizer.
        Only needed if you're using add_audio_chunk() manually instead of run_mic().
        """
        while self.running:
            try:
                data = self.audio_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            self.recognizer.AcceptWaveform(data)

        # Flush any remaining data
        self.audio_queue.queue.clear()


# Example usage:
if __name__ == "__main__":
    """
    1) Provide model_path and speaker_model_path to VoskSpeechRecognizer.
    2) Optionally set grammar to constrain recognized phrases.
    3) Use run_mic() to capture live audio from your microphone.
    """
    model_path = "/opt/vosk_model/speech_model"
    speaker_model_path = "/opt/vosk_model/speaker_model"  # Or None if not available

    recognizer = VoskSpeechRecognizer(
        model_path=model_path,
        sample_rate=48000.0,
        speaker_model_path=speaker_model_path
    )

    # Optional grammar
    my_grammar = ["hello world", "how are you", "turn on the light", "turn off the light"]
    recognizer.set_grammar(my_grammar)

    # Capture mic and block until Ctrl+C
    recognizer.run_mic(block_size=8000, show_partial=True)
    print("Done.")

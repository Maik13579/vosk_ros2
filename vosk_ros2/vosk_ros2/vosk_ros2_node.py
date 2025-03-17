#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
import threading
import time
import json

# Import your recognizer and JSON parser.
from vosk_ros2.speech_recognition import VoskSpeechRecognizer
from vosk_ros2.json_parser import parse_vosk_json
from vosk_ros2.grammar_parser import parse_grammar_lines
from vosk_ros2_interfaces.msg import RecognizedSpeech
from vosk_ros2_interfaces.action import SpeechDetection
from vosk_ros2_interfaces.srv import SetGrammar


class VoskNode(Node):
    def __init__(self):
        super().__init__('vosk_ros2')
        # Declare parameters.
        self.declare_parameter('model_path', '/opt/vosk_model/speech_model')
        self.declare_parameter('speaker_model_path', '/opt/vosk_model/speaker_model')
        self.declare_parameter('sample_rate', 48000.0)
        self.declare_parameter('block_size', 16000)
        self.declare_parameter('silence_timeout', 5.0)
        self.declare_parameter('max_alternatives', 0)
        self.declare_parameter('grammar_file', '')

        # Get parameter values.
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        speaker_model_path = self.get_parameter('speaker_model_path').get_parameter_value().string_value
        sample_rate = self.get_parameter('sample_rate').get_parameter_value().double_value
        block_size = self.get_parameter('block_size').get_parameter_value().integer_value
        silence_timeout = self.get_parameter('silence_timeout').get_parameter_value().double_value
        max_alternatives = self.get_parameter('max_alternatives').get_parameter_value().integer_value
        if max_alternatives < 1:
            max_alternatives = None # Disable
        grammar_file = self.get_parameter('grammar_file').get_parameter_value().string_value

        if grammar_file != '':
            with open(grammar_file, 'r') as f:
                grammar_lines = f.read().splitlines()
            grammar = parse_grammar_lines(grammar_lines)
            self.recognizer.set_grammar(grammar)

        self.block_size = block_size
        self.silence_timeout = silence_timeout

        self.get_logger().info(
            f"Initializing recognizer with model: {model_path}, speaker: {speaker_model_path}, sample_rate: {sample_rate}"
        )
        # Initialize VoskSpeechRecognizer.
        self.recognizer = VoskSpeechRecognizer(model_path, sample_rate, speaker_model_path, max_alternatives)

        # Create an Action Server for SpeechDetection.
        self._action_server = ActionServer(
            self,
            SpeechDetection,
            'speech_detection',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        # Create a Service for grammar setting.
        self._grammar_service = self.create_service(SetGrammar, 'set_grammar', self.grammar_callback)


    def goal_callback(self, goal_request):
        self.get_logger().info("Received new speech detection goal.")
        return rclpy.action.GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Canceling speech detection goal.")
        self.recognizer.stop_recognition()
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info("Starting speech recognition.")
        continuous = goal_handle.request.continuous
        publish_partial = goal_handle.request.publish_partial

        if continuous:
            # Continuous mode: repeatedly capture utterances.
            while not goal_handle.is_cancel_requested:
                self.get_logger().info("Capturing new utterance...")
                self.recognizer.run_mic(
                    block_size=self.block_size,
                    silence_timeout=self.silence_timeout
                )
                final_json = self.recognizer.get_final_result()
                self.get_logger().info("Utterance complete. Final JSON: " + final_json)
                final_msg = parse_vosk_json(final_json)
                feedback = SpeechDetection.Feedback()
                feedback.continuous_result = final_msg
                feedback.partial_text = final_msg.text
                goal_handle.publish_feedback(feedback)
            self.get_logger().info("Continuous mode canceled by client.")
            goal_handle.succeed()
            return SpeechDetection.Result(
                final_result=RecognizedSpeech(), success=True, message="Canceled"
            )
        else:
            # Single mode: run recognition once and send partial feedback while waiting.
            recog_thread = threading.Thread(
                target=self._run_recognition, daemon=True
            )
            recog_thread.start()
            while recog_thread.is_alive():
                partial_json = self.recognizer.get_partial_result()
                try:
                    partial_data = json.loads(partial_json)
                    text = partial_data.get("partial", "")
                except Exception:
                    text = ""
                if publish_partial and text:
                    feedback = SpeechDetection.Feedback()
                    continuous_result = RecognizedSpeech()
                    continuous_result.text = text
                    feedback.continuous_result = continuous_result
                    feedback.partial_text = text
                    goal_handle.publish_feedback(feedback)
                time.sleep(0.5)
                if goal_handle.is_cancel_requested:
                    self.get_logger().info("Goal canceled.")
                    self.recognizer.stop_recognition()
                    recog_thread.join()
                    goal_handle.succeed() 
                    return SpeechDetection.Result(
                        final_result=RecognizedSpeech(), success=False, message="Canceled"
                    )
            recog_thread.join()
            time.sleep(0.5)
            final_json = self.recognizer.get_final_result()
            self.get_logger().info("Recognition complete. Final JSON: " + final_json)
            final_msg = parse_vosk_json(final_json)
            goal_handle.succeed() 
            return SpeechDetection.Result(
                final_result=final_msg, success=True, message=""
            )

    def _run_recognition(self):
        """
        Runs the microphone capture with silence detection.
        This function blocks until recognition stops.
        """
        self.recognizer.run_mic(
            block_size=self.block_size,
            silence_timeout=self.silence_timeout
        )

    def grammar_callback(self, request, response):
        self.get_logger().info("Received SetGrammar service request.")
        try:
            if request.path.strip():
                with open(request.path, "r") as f:
                    lines = f.readlines()
            else:
                lines = request.grammar
            if lines is None or len(lines) == 0:
                expansions = []
            # Expand the grammar using your parser.
            expansions, warnings = parse_grammar_lines(lines)
            if warnings:
                for warning in warnings:
                    self.get_logger().warn(warning)

            # Update the recognizer's grammar.
            self.recognizer.set_grammar(expansions)
            response.expansions = expansions if request.return_expansions else []
            response.success = True
            response.message = ""
        except Exception as e:
            response.expansions = []
            response.success = False
            response.message = str(e)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = VoskNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

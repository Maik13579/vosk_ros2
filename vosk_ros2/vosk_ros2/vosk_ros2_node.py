import rclpy
from rclpy.node import Node
from vosk_ros2.speech_recognition import VoskSpeechRecognizer
from vosk_ros2.parse_json import parse_vosk_json
from vosk_ros2_interfaces.msg import RecognizedSpeech
from vosk_ros2_interfaces.action import SpeechDetection
from rclpy.action import ActionServer, CancelResponse

class VoskNode(Node):
    def __init__(self):
        super().__init__('vosk_ros2')
        self.declare_parameter('model_path', '/models/vosk_model')
        self.declare_parameter('sample_rate', 16000.0)
        
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        sample_rate = self.get_parameter('sample_rate').get_parameter_value().double_value

        self.recognizer = VoskSpeechRecognizer(model_path, sample_rate)
        self.server = ActionServer(
            self,
            SpeechDetection,
            'speech_detection',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def goal_callback(self, goal_request):
        self.get_logger().info('Received new SpeechDetection goal')
        return rclpy.action.GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Canceling SpeechDetection goal')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Starting speech detection...')
        self.recognizer.start_recognition()

        # For demo, just wait a few seconds, gather result, and finish
        # In real usage, you'd feed audio chunks from a microphone subscription
        await self._fake_audio_input()

        final_json = self.recognizer.get_final_result()
        final_msg = parse_vosk_json(final_json)
        goal_handle.succeed()
        return SpeechDetection.Result(final_result=final_msg, success=True, error_message='')

    async def _fake_audio_input(self):
        # This is a placeholder for streaming audio from a mic or .wav
        import asyncio
        # For demonstration, just sleep
        await asyncio.sleep(5)
        self.recognizer.stop_recognition()

def main(args=None):
    rclpy.init(args=args)
    node = VoskNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

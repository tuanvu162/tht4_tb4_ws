"""Push-to-talk Vietnamese Whisper input; audio files and text for repeatable demos."""

import argparse
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String


def publish_command(node, publisher, text, timeout=5.0):
    text = text.strip()
    if not text:
        node.get_logger().warning('Không nhận được câu nói; không gửi lệnh.')
        return False
    deadline = time.monotonic() + timeout
    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() == 0:
        node.get_logger().error('Không có node nhận /voice_command. Chạy tht7.launch.py trước.')
        return False
    publisher.publish(String(data=text))
    # Allow middleware to transmit before a one-shot publisher exits.
    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info(f'Đã gửi: {text}')
    return True


def transcribe(model, audio):
    """Audio ndarray must be mono float32 at Whisper's required 16 kHz."""
    if isinstance(audio, np.ndarray):
        if audio.size == 0 or not np.isfinite(audio).all():
            raise ValueError('Bản thu rỗng hoặc chứa mẫu âm thanh không hợp lệ')
        if np.sqrt(np.mean(np.square(audio))) < 0.003:
            return ''
    result = model.transcribe(audio, language='vi', task='transcribe', fp16=False,
                              temperature=0, condition_on_previous_text=False,
                              verbose=False)
    return result.get('text', '').strip()


def text_main(args=None):
    parser = argparse.ArgumentParser(description='Gửi câu lệnh văn bản để thử Nav2.')
    parser.add_argument('text')
    options = parser.parse_args(remove_ros_args(args=args)[1:])
    rclpy.init(args=args)
    node = Node('text_command_input')
    try:
        publisher = node.create_publisher(String, '/voice_command', 10)
        return 0 if publish_command(node, publisher, options.text) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default='small', help='Multilingual Whisper model or .pt path')
    parser.add_argument('--model-dir', default=str(Path.home() / '.cache/whisper'))
    parser.add_argument('--audio', help='Transcribe one audio file (requires ffmpeg)')
    parser.add_argument('--seconds', type=float, default=5.0)
    parser.add_argument('--device', help='Microphone device index or name')
    parser.add_argument('--compute-device', default='cpu', choices=['cpu', 'cuda'])
    parser.add_argument('--list-devices', action='store_true')
    options = parser.parse_args(remove_ros_args(args=args)[1:])
    if not math.isfinite(options.seconds) or not 0.5 <= options.seconds <= 30:
        parser.error('--seconds must be between 0.5 and 30')
    if options.list_devices:
        import sounddevice as sd
        print(sd.query_devices())
        return 0
    if options.audio and not Path(options.audio).expanduser().is_file():
        parser.error(f'Audio file does not exist: {options.audio}')
    if options.model.endswith('.en'):
        parser.error('Use a multilingual model for Vietnamese (small, base, medium, ...).')
    try:
        import whisper
        print(f'Đang nạp Whisper {options.model}; lần đầu cần tải model. Ctrl+C để thoát.')
        model = whisper.load_model(options.model, device=options.compute_device,
                                   download_root=str(Path(options.model_dir).expanduser()))
    except ImportError as error:
        parser.exit(1, f'Thiếu thư viện: {error}. Xem README, phần cài nhận giọng nói.\n')
    rclpy.init(args=args)
    node = Node('speech_input')
    publisher = node.create_publisher(String, '/voice_command', 10)
    try:
        if options.audio:
            audio = whisper.load_audio(str(Path(options.audio).expanduser()))
            text = transcribe(model, audio)
            return 0 if publish_command(node, publisher, text) else 1
        import sounddevice as sd
        device = options.device
        if device and device.isdigit():
            device = int(device)
        while rclpy.ok():
            answer = input('Nhấn Enter rồi nói trong %.1f giây (q để thoát): ' % options.seconds)
            if answer.strip().lower() in ('q', 'quit', 'exit'):
                break
            print('Đang thu...')
            audio = sd.rec(int(options.seconds * 16000), samplerate=16000,
                           channels=1, dtype='float32', device=device)
            sd.wait()
            text = transcribe(model, audio.reshape(-1))
            publish_command(node, publisher, text)
    except (KeyboardInterrupt, EOFError):
        pass
    except Exception as error:
        node.get_logger().error(f'Lỗi thu/nhận dạng giọng nói: {error}')
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

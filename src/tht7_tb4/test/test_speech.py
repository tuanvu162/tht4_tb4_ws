"""Reject silent/invalid recordings before calling speech recognition."""
# flake8: noqa: E402

from unittest.mock import Mock

import numpy as np
import pytest

pytest.importorskip('rclpy')
from tht7_tb4.speech_input import transcribe


def test_silence_cannot_create_a_navigation_command():
    model = Mock()
    assert transcribe(model, np.zeros(16000, dtype=np.float32)) == ''
    model.transcribe.assert_not_called()


@pytest.mark.parametrize('audio', [np.array([]), np.array([np.nan]), np.array([np.inf])])
def test_invalid_audio(audio):
    model = Mock()
    with pytest.raises(ValueError):
        transcribe(model, audio)
    model.transcribe.assert_not_called()


def test_uses_vietnamese_transcription():
    model = Mock()
    model.transcribe.return_value = {'text': ' Đi đến phòng học. '}
    audio = np.full(16000, 0.1, dtype=np.float32)
    assert transcribe(model, audio) == 'Đi đến phòng học.'
    assert model.transcribe.call_args.kwargs['language'] == 'vi'
    assert model.transcribe.call_args.kwargs['task'] == 'transcribe'
    assert model.transcribe.call_args.kwargs['fp16'] is False

"""Exercise Vietnamese grammar, unknown regions and ambiguous utterances."""

from pathlib import Path

import pytest

from tht7_tb4.commands import CommandError, parse_command
from tht7_tb4.regions import load_regions


CONFIG = Path(__file__).resolve().parents[1] / 'config/regions.yaml'


@pytest.mark.parametrize('text, route', [
    ('Đi đến khu vực A.', ('A',)),
    ('Di chuyển từ khu vực A đến khu vực B.', ('A', 'B')),
    ('Đi đến phòng thí nghiệm.', ('B',)),
    ('Đi đến hành lang', ('C',)),
    ('di chuyen tu phong hoc den hanh lang', ('A', 'C')),
    ('Robot hãy đi đến khu vực bê!', ('B',)),
    ('Đi đến xê', ('C',)),
    ('Di chuyển từ A đến A', ('A',)),
])
def test_valid_commands(text, route):
    assert parse_command(text, load_regions(CONFIG)[0]).route == route


@pytest.mark.parametrize('text', [
    'Đi đến khu vực Z', 'Di chuyển từ Z đến A', 'Di chuyển từ A đến Z',
    'Đi đến A rồi đến Z', 'Đi đến A B', 'Đi đến khu vực AB', '',
    'không đi đến A', 'đừng đi đến A', 'phòng học', 'Đi đến A và B',
    'Di chuyển từ A đến B đến C',
])
def test_invalid_commands_do_not_dispatch_partial_routes(text):
    with pytest.raises(CommandError):
        parse_command(text, load_regions(CONFIG)[0])


@pytest.mark.parametrize('text', ['Dừng lại!', 'dung', 'Hủy điều hướng', 'Robot dừng lại'])
def test_stop(text):
    command = parse_command(text, load_regions(CONFIG)[0])
    assert command.stop and not command.route

"""Strict command grammar: never drop an unknown source or destination."""

from dataclasses import dataclass
import re

from tht7_tb4.regions import normalize


class CommandError(ValueError):
    """A command cannot be mapped to a complete known route."""


@dataclass(frozen=True)
class Command:
    route: tuple = ()
    stop: bool = False


def parse_command(text, regions):
    sentence = normalize(text)
    if sentence in ('dung', 'dung lai', 'robot dung lai', 'huy', 'huy dieu huong'):
        return Command(stop=True)
    # Full matching prevents 'A rồi đến Z' from silently becoming a trip to A.
    sentence = re.sub(r'^(?:robot |hay |vui long )+', '', sentence)
    match = re.fullmatch(r'(?:di chuyen|di|den)(?: den)? (.+)', sentence)
    if not match:
        raise CommandError('Câu lệnh chưa rõ. Ví dụ: Đi đến khu vực A.')
    target = match.group(1)
    if target.startswith('tu '):
        parts = target[3:].split(' den ')
        if len(parts) != 2:
            raise CommandError('Dùng mẫu: Di chuyển từ khu vực A đến khu vực B.')
    else:
        parts = [target]
    aliases = {alias: region.id for region in regions for alias in region.aliases}
    route = []
    for part in parts:
        identifier = aliases.get(part)
        if identifier is None:
            names = ', '.join(f'{region.id} ({region.name})' for region in regions)
            raise CommandError(f'Khu vực không tồn tại hoặc chưa rõ: "{part}". Có: {names}.')
        if not route or route[-1] != identifier:
            route.append(identifier)
    return Command(tuple(route))

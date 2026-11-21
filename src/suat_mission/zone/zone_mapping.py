"""图案 <-> 收取区(B/C/D) 的对齐逻辑（纯 Python，不依赖 OpenCV/ROS）。

比赛约定：
  * 分类区(Classification_area) 地面上从左到右贴着三个二维码，依次对应 B / C / D 区；
  * 工作台上有三个方块，每个方块顶面印着一个图案（clock / teddybear / banana / cake ...）；
  * 图案的左右顺序，就是它要去的收取区顺序 —— 左1图案去左1二维码那个区。

所以核心只有一句话：**按下标配对**。
    patterns[i]  ->  markers[i]  (两者都按图像 x 从小到大排好)

本模块只做这件事，并负责把二维码解出来的文本（"B"、"B区"、"Collection_B" ...）
归一成 'B'/'C'/'D'，以及在解码失败时按位置约定回退，同时把回退这件事记录下来，
避免上层把"猜的"当成"读到的"。

用法：
    from zone_mapping import map_patterns_to_zones, format_report

    result = map_patterns_to_zones(['clock', 'teddybear', 'banana'], ['B', 'C', 'D'])
    print(format_report(result))
    result.zone_of          # {'clock': 'B', 'teddybear': 'C', 'banana': 'D'}
"""
import json
import re
from collections import Counter
from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Sequence

ZONES = ('B', 'C', 'D')          # 左 -> 右 的固定顺序
UNKNOWN = 'unknown'              # 检测没认出来的槽位占位符
ZONE_NAV = {                     # 收费区导航点名，接入底盘时直接用
    'B': 'Collection_B',
    'C': 'Collection_C',
    'D': 'Collection_D',
}

_ZONE_RE = re.compile(r'(?<![A-Z])([BCD])(?![A-Z])')


def normalize_zone(text: Optional[str]) -> Optional[str]:
    """把二维码解码出的文本归一成 'B'/'C'/'D'，认不出返回 None。

    能认：'B' / 'b' / 'B区' / 'zone-b' / 'Collection_B' / 'collection b' / 'B:xxx'
    认不出（或一句话里出现两个不同字母）返回 None，绝不去猜。
    """
    if not text:
        return None
    found = set(_ZONE_RE.findall(str(text).upper()))
    return found.pop() if len(found) == 1 else None


@dataclass(frozen=True)
class Slot:
    """一个方块的结论。index=0/1/2 对应图像里的左/中/右。"""
    index: int
    pattern: str
    zone: str
    marker_text: Optional[str] = None      # 二维码原始文本（没解出来就是 None）
    zone_source: str = 'position'          # 'qr' = 真的从二维码读到的；'position' = 按左->右约定

    @property
    def nav_point(self) -> str:
        return ZONE_NAV[self.zone]

    @property
    def position_name(self) -> str:
        return ('左', '中', '右')[self.index]


@dataclass
class Alignment:
    slots: List[Slot] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    @property
    def zone_of(self) -> dict:
        """{图案代号: 区域}，最常用的形式。"""
        return {s.pattern: s.zone for s in self.slots}

    @property
    def nav_order(self) -> List[str]:
        """按 左->右 给出要去的导航点名。"""
        return [s.nav_point for s in self.slots]

    def as_dict(self) -> dict:
        return {
            'zones': [s.zone for s in self.slots],
            'patterns': [s.pattern for s in self.slots],
            'mapping': [{'pattern': s.pattern, 'zone': s.zone,
                         'position': s.position_name,
                         'marker_text': s.marker_text,
                         'zone_source': s.zone_source} for s in self.slots],
            'zone_of': self.zone_of,
            'warnings': self.warnings,
        }

    def as_json(self) -> str:
        return json.dumps(self.as_dict(), ensure_ascii=False, indent=2)


def map_patterns_to_zones(patterns: Sequence[str],
                          marker_texts: Optional[Sequence[Optional[str]]] = None,
                          require_unique: bool = True,
                          source: str = 'qr') -> Alignment:
    """把图案（必须已按左->右排好）映射到 B/C/D。

    参数
      patterns     : 三个图案代号，左->右，例如 ['clock','teddybear','banana']
      marker_texts : 三个二维码的原始解码文本，左->右；没解出来就传 None / 空列表
      require_unique: 图案代号是否必须互不相同（同一种图案出现两次说明检测串了）
      source       : 二维码这条信息是怎么来的，写进报告用
                     'qr' = 真解码出文本；'fingerprint' = 解不出但和指纹库对上了

    规则
      1. 两个列表都按下标配对，不重排、不猜。
      2. 二维码文本能归一成 B/C/D 且恰好是 B/C/D 的一个排列 -> 用它，zone_source='qr'。
      3. 解不出 / 解重了 / 不是 BCD 排列 -> 回退到位置约定（左=B, 中=C, 右=D），
         zone_source='position'，并在 warnings 里写清楚原因。
    """
    patterns = [str(p).strip() for p in patterns]
    if len(patterns) != 3:
        raise ValueError('需要正好 3 个图案，实际 %d 个：%r' % (len(patterns), patterns))
    if any(not p for p in patterns):
        raise ValueError('图案代号不能为空：%r' % (patterns,))
    repeated = [p for p, count in Counter(patterns).items() if count > 1 and p != UNKNOWN]
    if require_unique and repeated:
        raise ValueError('三个图案代号必须互不相同，实际 %r（检测可能把同一种图案识别了两次）' % (patterns,))

    warnings: List[str] = []
    missing = [i for i, p in enumerate(patterns) if p == UNKNOWN]
    if missing:
        warnings.append('%s 槽位没有认出图案，该槽位的区域结论只是位置推断'
                        % '/'.join(('左', '中', '右')[i] for i in missing))
    texts: List[Optional[str]] = list(marker_texts or [])
    if len(texts) < 3:
        texts += [None] * (3 - len(texts))
    texts = [str(t).strip() if t not in (None, '') else None for t in texts[:3]]

    decoded = [normalize_zone(t) for t in texts]
    zones: Sequence[str]
    if all(decoded) and sorted(decoded) == list(ZONES):
        zones = decoded                                    # type: ignore[assignment]
        zone_source = source
    else:
        zones = ZONES
        zone_source = 'position'
        if all(t is None for t in texts):
            warnings.append('三个二维码都没有解码文本，按"左->右 = B/C/D"的位置约定配对')
        elif len(decoded) == 3 and any(z is None for z in decoded) and all(decoded):
            warnings.append('二维码解密结果重复，按位置约定配对：%r' % (decoded,))
        else:
            warnings.append('二维码解码结果不是 B/C/D 的一个排列（%r），按位置约定配对' % (decoded,))

    slots = [Slot(i, patterns[i], zones[i], texts[i], zone_source) for i in range(3)]
    return Alignment(slots, warnings)


def _pad(text: str, width: int) -> str:
    """按显示宽度补空格（中文算 2 个字符），否则表格在终端里会歪。"""
    shown = sum(2 if ord(ch) > 127 else 1 for ch in text)
    return text + ' ' * max(0, width - shown)


def format_report(result: Alignment, title: str = '图案 -> 区域') -> str:
    """人看的表格 + 一行结论。"""
    lines = [title]
    header = [('位置', 6), ('图案代号', 14), ('区域', 6), ('依据', 10)]
    lines.append('  ' + ''.join(_pad(text, width) for text, width in header) + '二维码原文')
    lines.append('  ' + '-' * 58)
    for s in result.slots:
        src = {'qr': '二维码', 'fingerprint': '指纹'}.get(s.zone_source, '位置约定')
        lines.append('  ' + _pad(s.position_name, 6) + _pad(s.pattern, 14)
                     + _pad(s.zone, 6) + _pad(src, 10) + (s.marker_text or '-'))
    lines.append('  结论: ' + ' | '.join('%s -> %s区' % (s.pattern, s.zone) for s in result.slots))
    lines.append('  导航: ' + ' -> '.join(result.nav_order))
    for w in result.warnings:
        lines.append('  注意: ' + w)
    return '\n'.join(lines)


if __name__ == '__main__':
    demo = map_patterns_to_zones(['clock', 'teddybear', 'banana'], ['B', 'C', 'D'])
    print(format_report(demo))
    print()
    print(format_report(map_patterns_to_zones(['teddybear', 'banana', 'clock'], [])))

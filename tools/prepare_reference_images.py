"""Extract example templates from the team's block photographs.

These crops are initial references, not a substitute for robot-camera training data.
The separate board photograph is deliberately not used to prepare the templates.
"""
from pathlib import Path
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
SOURCES_DIR = ROOT / 'assets' / 'source_images'
OUTPUT = ROOT / 'src' / 'suat_mission' / 'assets' / 'templates'
SOURCES = {
    # Bounding boxes are (left, top, right, bottom) in the original photographs.
    'teddybear': ('teddybear.jpg', (300, 380, 900, 1100)),
    'clock': ('clock.jpg', (260, 390, 970, 1160)),
    'banana': ('banana.jpg', (270, 510, 1000, 1210)),
    'cake': ('cake.jpg', (540, 220, 1370, 1050)),
    # The rule illustration is an alternative, independent cake reference.
    'cake_rule_reference': ('cake_rule_reference.jpg', (0, 0, 444, 418)),
}


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    for label, (name, box) in SOURCES.items():
        with Image.open(SOURCES_DIR / name) as original:
            crop = original.convert('RGB').crop(box)
            crop.thumbnail((180, 180), Image.Resampling.LANCZOS)
            crop.save(OUTPUT / (label + '.png'))
        print(label, name, box, '->', OUTPUT / (label + '.png'))


if __name__ == '__main__':
    main()

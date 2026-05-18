import re
import math

with open("config/sim_params.yaml", 'r', encoding='utf-8') as f:
    content = f.read()

angle_deg = 30
angle_rad = math.radians(angle_deg)

content = re.sub(
    r'(name:\s*"shinkansen_front".*?relative_rpy:\s*\[\s*[-\d\.]+,\s*[-\d\.]+,\s*)[-\d\.]+(\s*\])',
    rf'\g<1>{angle_rad:.4f}\g<2>',
    content,
    flags=re.DOTALL
)
content = re.sub(
    r'(name:\s*"shinkansen_mid".*?relative_rpy:\s*\[\s*[-\d\.]+,\s*[-\d\.]+,\s*)[-\d\.]+(\s*\])',
    rf'\g<1>{angle_rad:.4f}\g<2>',
    content,
    flags=re.DOTALL
)
content = re.sub(
    r'(name:\s*"shinkansen_rear".*?relative_rpy:\s*\[\s*[-\d\.]+,\s*[-\d\.]+,\s*)[-\d\.]+(\s*\])',
    rf'\g<1>{angle_rad:.4f}\g<2>',
    content,
    flags=re.DOTALL
)

print(re.findall(r'name: "shinkansen_.*?\n.*?relative_rpy: \[.*?\]', content, re.DOTALL))

import csv

path = "sim_results/sweep_20260605_161315/sweep_summary.csv"
data = {}
with open(path) as f:
    r = csv.DictReader(f)
    for row in r:
        ang = float(row['antenna_angle'])
        name = row['vehicle_name']
        if ang not in data:
            data[ang] = {}
        
        data[ang][name] = {
            'data': float(row['total_data_MB']) if row['total_data_MB'] else 0.0,
            'time': float(row['connected_time_s']) if row['connected_time_s'] else 0.0,
            'tp': float(row['average_throughput_Gbps']) if row['average_throughput_Gbps'] else 0.0
        }

print("| 角度 (deg) | Front (Gbps) | Mid (Gbps) | Rear (Gbps) |")
print("| :---: | :---: | :---: | :---: |")
for ang in sorted(data.keys()):
    f = data[ang].get('shinkansen_front', {'tp': 0.0})
    m = data[ang].get('shinkansen_mid', {'tp': 0.0})
    r = data[ang].get('shinkansen_rear', {'tp': 0.0})
    print(f"| {ang}° | {f['tp']:.3f} | {m['tp']:.3f} | {r['tp']:.3f} |")

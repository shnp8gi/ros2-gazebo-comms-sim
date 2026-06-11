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
        
        if name != 'shinkansen_total':
            data[ang][name] = {
                'data': float(row['total_data_MB']),
                'time': float(row['connected_time_s']),
                'tp': float(row['average_throughput_Gbps'])
            }
        else:
            data[ang][name] = {
                'data': float(row['total_data_MB'])
            }

print("| 角度 (deg) | Front (Gbps) | Mid (Gbps) | Rear (Gbps) | Total Average (Gbps) |")
print("| :---: | :---: | :---: | :---: | :---: |")
for ang in sorted(data.keys()):
    f = data[ang].get('shinkansen_front', {'tp': 0.0, 'data': 0.0, 'time': 0.0})
    m = data[ang].get('shinkansen_mid', {'tp': 0.0, 'data': 0.0, 'time': 0.0})
    r = data[ang].get('shinkansen_rear', {'tp': 0.0, 'data': 0.0, 'time': 0.0})
    tot_data = data[ang]['shinkansen_total']['data']
    tot_time = f['time'] + m['time'] + r['time']
    tot_tp = (tot_data * 8.0 / 1000.0 / tot_time) if tot_time > 0 else 0.0
    print(f"| {ang}° | {f['tp']:.3f} | {m['tp']:.3f} | {r['tp']:.3f} | {tot_tp:.3f} |")

#!/usr/bin/env python3
import os
import sys
import json
import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Define color palette (elegant, modern, high contrast)
COLORS = {
    'grid': '#333333',
    'bg': '#121212',
    'card_bg': '#1e1e1e',
    'text': '#ffffff',
    'text_muted': '#aaaaaa'
}

def clean_label(name):
    """Convert name like 'shinkansen_front' to 'Front (Shinkansen)' or 'shinkansen_total' to 'Shinkansen Total'"""
    if not name:
        return ""
    name_str = str(name)
    if name_str.endswith('_total'):
        prefix = name_str[:-6]
        return f"{prefix.capitalize()} Total"
    
    if '_' in name_str:
        parts = name_str.split('_')
        entity = parts[0].capitalize()
        antenna = " ".join([p.capitalize() for p in parts[1:]])
        return f"{antenna} ({entity})"
        
    return name_str.capitalize()

def get_vehicle_colors(df):
    """Dynamically assign colors to vehicles and their total entries based on the dataframe."""
    unique_names = df['vehicle_name'].unique()
    
    # Separate totals and individual antennas
    totals = sorted([v for v in unique_names if str(v).endswith('_total')])
    individuals = sorted([v for v in unique_names if not str(v).endswith('_total')])
    
    color_map = {}
    
    # Preset colors for individuals (modern high contrast palette)
    preset_indivs = [
        '#00bcd4',  # Teal / Cyan
        '#e91e63',  # Pink / Magenta
        '#ff9800',  # Amber / Orange
        '#4caf50',  # Green
        '#9c27b0',  # Purple
        '#03a9f4',  # Light Blue
        '#ff5722',  # Deep Orange
        '#e81e63',  # Rose
    ]
    
    for i, name in enumerate(individuals):
        color_map[name] = preset_indivs[i % len(preset_indivs)]
        
    # Preset colors for totals
    preset_totals = [
        '#3f51b5',  # Royal Blue
        '#6366f1',  # Indigo
        '#818cf8',  # Lavender
    ]
    for i, name in enumerate(totals):
        color_map[name] = preset_totals[i % len(preset_totals)]
        
    return color_map

def setup_matplotlib_style():
    """Configure matplotlib for academic-style plots matching user preferences"""
    plt.rcParams.update({
        'figure.facecolor': 'white',
        'axes.facecolor': 'white',
        'savefig.facecolor': 'white',
        'axes.edgecolor': 'black',
        'axes.grid': False,  # Managed manually
        'xtick.color': 'black',
        'ytick.color': 'black',
        'axes.labelcolor': 'black',
        'axes.titlecolor': 'black',
        'figure.titlesize': 18,
        'axes.titlesize': 18,
        'axes.labelsize': 18,
        'xtick.labelsize': 18,
        'ytick.labelsize': 18,
        'legend.facecolor': 'white',
        'legend.edgecolor': 'black',
        'legend.fontsize': 14,
        'text.color': 'black',
        'font.family': 'Times New Roman',
        'figure.dpi': 150,
        'axes.prop_cycle': plt.cycler("color", ["blue", "orange", "green", "olive", "peru", "cyan", "purple"])
    })


def generate_static_plots(df, plots_dir):
    """Generate academic-style matplotlib plots and save them as PNGs"""
    setup_matplotlib_style()
    os.makedirs(plots_dir, exist_ok=True)
    
    unique_names = df['vehicle_name'].unique()
    total_names = sorted([v for v in unique_names if str(v).endswith('_total')])
    individual_names = sorted([v for v in unique_names if not str(v).endswith('_total')])
    
    # Sort dataframe by antenna_angle
    df_sorted = df.sort_values(by='antenna_angle')
    
    # We filter out 360.0 degrees from the continuous line plot if it's an outlier
    # and handle it as a separate baseline if needed.
    has_360 = 360.0 in df_sorted['antenna_angle'].values
    df_line = df_sorted[df_sorted['antenna_angle'] <= 180.0]
    
    # If df_line is empty, just use the entire df
    if df_line.empty:
        df_line = df_sorted
        
    angles = df_line['antenna_angle'].unique()
    
    # Get baseline values at 360 degrees if present
    baselines = {}
    if has_360:
        baseline_df = df_sorted[df_sorted['antenna_angle'] == 360.0]
        for t_name in total_names:
            b_val = baseline_df[baseline_df['vehicle_name'] == t_name]['total_data_MB'].values
            if len(b_val) > 0:
                baselines[t_name] = b_val[0]

    size_of_figure_ration = 7 / 13

    # Helper function to apply common academic plot styling
    def apply_academic_styling(ax, xlabel, ylabel, xmin, xmax, ymin=None):
        ax.set_xlabel(xlabel, fontname="Times New Roman")
        ax.set_ylabel(ylabel, fontname="Times New Roman")
        
        # Legend styling
        leg = ax.legend(loc='best',
                        facecolor='white',
                        edgecolor='black',
                        framealpha=1,
                        fancybox=False,
                        ncol=1)
        try:
            leg.set_draggable(True)
        except:
            pass
            
        # Grid styling
        ax.grid(which="major", color='black', linestyle='-', zorder=0)
        ax.minorticks_on()
        ax.grid(which="minor", axis='y', linestyle='--', color='lightgray', zorder=0)
        if ax.get_yscale() == 'log':
            ax.yaxis.set_minor_locator(plt.LogLocator(base=10, subs='all'))
            
        # Ticks direction
        ax.tick_params(direction='in', which='both')
        
        if xmin is not None and xmax is not None:
            ax.set_xlim(xmin, xmax)
            
        if ymin is not None:
            ax.set_ylim(bottom=ymin)

    # --- Plot 1: Total Data Transferred ---
    fig, ax = plt.subplots(figsize=(16, 9))
    
    # Plot individual antennas
    for vehicle in individual_names:
        v_df = df_line[df_line['vehicle_name'] == vehicle]
        if not v_df.empty:
            if 'theory' in vehicle.lower():
                line, = ax.plot(v_df['antenna_angle'], v_df['total_data_MB'], 
                        "-r", label=clean_label(vehicle), lw=2, zorder=3)
            else:
                line, = ax.plot(v_df['antenna_angle'], v_df['total_data_MB'], 
                        "-", label=clean_label(vehicle), linewidth=2)
            if 'total_data_MB_std' in v_df.columns and v_df['total_data_MB_std'].any():
                std_val = v_df['total_data_MB_std']
                ax.fill_between(v_df['antenna_angle'],
                                np.maximum(0, v_df['total_data_MB'] - std_val),
                                v_df['total_data_MB'] + std_val,
                                color=line.get_color(), alpha=0.15)
            
    # Plot total
    for t_name in total_names:
        total_df = df_line[df_line['vehicle_name'] == t_name]
        if not total_df.empty:
            if 'theory' in t_name.lower():
                line, = ax.plot(total_df['antenna_angle'], total_df['total_data_MB'], 
                        "-r", label=clean_label(t_name), lw=3, zorder=3)
            else:
                line, = ax.plot(total_df['antenna_angle'], total_df['total_data_MB'], 
                        "-", label=clean_label(t_name), linewidth=3)
            if 'total_data_MB_std' in total_df.columns and total_df['total_data_MB_std'].any():
                std_val = total_df['total_data_MB_std']
                ax.fill_between(total_df['antenna_angle'],
                                np.maximum(0, total_df['total_data_MB'] - std_val),
                                total_df['total_data_MB'] + std_val,
                                color=line.get_color(), alpha=0.15)
        
        # Plot baseline if available
        if t_name in baselines:
            ax.axhline(y=baselines[t_name], color='red', linestyle='--', alpha=0.8,
                       label=f'Baseline ({clean_label(t_name)} 360°): {baselines[t_name]:.1f} MB')

    xmin = angles.min() if len(angles) > 1 else None
    xmax = angles.max() if len(angles) > 1 else None
    apply_academic_styling(ax, 'Antenna Angle [degrees]', 'Total Data Transferred [MB]', xmin, xmax, ymin=0)
    fig.set_size_inches(16 * size_of_figure_ration, 9 * size_of_figure_ration)
    plt.tight_layout()
    plt.savefig(os.path.join(plots_dir, 'total_data.png'), dpi=300)
    plt.close()

    # --- Plot 2: Average Throughput ---
    fig, ax = plt.subplots(figsize=(16, 9))
    for vehicle in individual_names:
        v_df = df_line[df_line['vehicle_name'] == vehicle]
        if not v_df.empty:
            if 'theory' in vehicle.lower():
                line, = ax.plot(v_df['antenna_angle'], v_df['average_throughput_Gbps'], 
                        "-r", label=clean_label(vehicle), lw=2, zorder=3)
            else:
                line, = ax.plot(v_df['antenna_angle'], v_df['average_throughput_Gbps'], 
                        "-", label=clean_label(vehicle), linewidth=2)
            if 'average_throughput_Gbps_std' in v_df.columns and v_df['average_throughput_Gbps_std'].any():
                std_val = v_df['average_throughput_Gbps_std']
                ax.fill_between(v_df['antenna_angle'],
                                np.maximum(0.0, v_df['average_throughput_Gbps'] - std_val),
                                v_df['average_throughput_Gbps'] + std_val,
                                color=line.get_color(), alpha=0.15)
                
    for t_name in total_names:
        total_df = df_line[df_line['vehicle_name'] == t_name]
        if not total_df.empty:
            if 'theory' in t_name.lower():
                line, = ax.plot(total_df['antenna_angle'], total_df['average_throughput_Gbps'], 
                        "-r", label=clean_label(t_name), lw=3, zorder=3)
            else:
                line, = ax.plot(total_df['antenna_angle'], total_df['average_throughput_Gbps'], 
                        "-", label=clean_label(t_name), linewidth=3)
            if 'average_throughput_Gbps_std' in total_df.columns and total_df['average_throughput_Gbps_std'].any():
                std_val = total_df['average_throughput_Gbps_std']
                ax.fill_between(total_df['antenna_angle'],
                                np.maximum(0.0, total_df['average_throughput_Gbps'] - std_val),
                                total_df['average_throughput_Gbps'] + std_val,
                                color=line.get_color(), alpha=0.15)
            
    apply_academic_styling(ax, 'Antenna Angle [degrees]', 'Average Throughput [Gbps]', xmin, xmax, ymin=0)
    fig.set_size_inches(16 * size_of_figure_ration, 9 * size_of_figure_ration)
    plt.tight_layout()
    plt.savefig(os.path.join(plots_dir, 'throughput.png'), dpi=300)
    plt.close()

    # --- Plot 3: Average RSSI ---
    fig, ax = plt.subplots(figsize=(16, 9))
    for vehicle in individual_names:
        v_df = df_line[df_line['vehicle_name'] == vehicle]
        if not v_df.empty:
            if 'theory' in vehicle.lower():
                line, = ax.plot(v_df['antenna_angle'], v_df['average_rssi_dBm'], 
                        "-r", label=clean_label(vehicle), lw=2, zorder=3)
            else:
                line, = ax.plot(v_df['antenna_angle'], v_df['average_rssi_dBm'], 
                        "-", label=clean_label(vehicle), linewidth=2)
            if 'average_rssi_dBm_std' in v_df.columns and v_df['average_rssi_dBm_std'].any():
                std_val = v_df['average_rssi_dBm_std']
                ax.fill_between(v_df['antenna_angle'],
                                v_df['average_rssi_dBm'] - std_val,
                                v_df['average_rssi_dBm'] + std_val,
                                color=line.get_color(), alpha=0.15)
                
    for t_name in total_names:
        total_df = df_line[df_line['vehicle_name'] == t_name]
        if not total_df.empty:
            if 'theory' in t_name.lower():
                line, = ax.plot(total_df['antenna_angle'], total_df['average_rssi_dBm'], 
                        "-r", label=clean_label(t_name), lw=3, zorder=3)
            else:
                line, = ax.plot(total_df['antenna_angle'], total_df['average_rssi_dBm'], 
                        "-", label=clean_label(t_name), linewidth=3)
            if 'average_rssi_dBm_std' in total_df.columns and total_df['average_rssi_dBm_std'].any():
                std_val = total_df['average_rssi_dBm_std']
                ax.fill_between(total_df['antenna_angle'],
                                total_df['average_rssi_dBm'] - std_val,
                                total_df['average_rssi_dBm'] + std_val,
                                color=line.get_color(), alpha=0.15)
            
    # Add a horizontal line at the operational threshold if known (-68.5 dBm)
    ax.axhline(y=-68.5, color='red', linestyle=':', alpha=0.8, label='MCS Threshold (-68.5 dBm)')
    
    apply_academic_styling(ax, 'Antenna Angle [degrees]', 'Average RSSI [dBm]', xmin, xmax)
    fig.set_size_inches(16 * size_of_figure_ration, 9 * size_of_figure_ration)
    plt.tight_layout()
    plt.savefig(os.path.join(plots_dir, 'rssi.png'), dpi=300)
    plt.close()

    # --- Plot 4: Connected Time ---
    fig, ax = plt.subplots(figsize=(16, 9))
    for vehicle in individual_names:
        v_df = df_line[df_line['vehicle_name'] == vehicle]
        if not v_df.empty:
            if 'theory' in vehicle.lower():
                line, = ax.plot(v_df['antenna_angle'], v_df['connected_time_s'], 
                        "-r", label=clean_label(vehicle), lw=2, zorder=3)
            else:
                line, = ax.plot(v_df['antenna_angle'], v_df['connected_time_s'], 
                        "-", label=clean_label(vehicle), linewidth=2)
            if 'connected_time_s_std' in v_df.columns and v_df['connected_time_s_std'].any():
                std_val = v_df['connected_time_s_std']
                ax.fill_between(v_df['antenna_angle'],
                                np.maximum(0, v_df['connected_time_s'] - std_val),
                                v_df['connected_time_s'] + std_val,
                                color=line.get_color(), alpha=0.15)
                
    for t_name in total_names:
        total_df = df_line[df_line['vehicle_name'] == t_name]
        if not total_df.empty:
            if 'theory' in t_name.lower():
                line, = ax.plot(total_df['antenna_angle'], total_df['connected_time_s'], 
                        "-r", label=clean_label(t_name), lw=3, zorder=3)
            else:
                line, = ax.plot(total_df['antenna_angle'], total_df['connected_time_s'], 
                        "-", label=clean_label(t_name), linewidth=3)
            if 'connected_time_s_std' in total_df.columns and total_df['connected_time_s_std'].any():
                std_val = total_df['connected_time_s_std']
                ax.fill_between(total_df['antenna_angle'],
                                np.maximum(0, total_df['connected_time_s'] - std_val),
                                total_df['connected_time_s'] + std_val,
                                color=line.get_color(), alpha=0.15)
            
    apply_academic_styling(ax, 'Antenna Angle [degrees]', 'Connected Time [seconds]', xmin, xmax, ymin=0)
    fig.set_size_inches(16 * size_of_figure_ration, 9 * size_of_figure_ration)
    plt.tight_layout()
    plt.savefig(os.path.join(plots_dir, 'connected_time.png'), dpi=300)
    plt.close()

def generate_html_report(df, output_path, sweep_id):
    """Generate a premium, self-contained HTML interactive dashboard"""
    unique_names = df['vehicle_name'].unique()
    total_names = [v for v in unique_names if str(v).endswith('_total')]
    individual_names = [v for v in unique_names if not str(v).endswith('_total')]
    
    # Pick the first total or first individual if no totals exist
    primary_total_name = total_names[0] if total_names else (individual_names[0] if individual_names else '')
    
    if primary_total_name:
        total_data_df = df[df['vehicle_name'] == primary_total_name]
    else:
        total_data_df = pd.DataFrame()
    
    # Filter 360 out of calculations for "optimal sweep angle" to make it representative
    if not total_data_df.empty:
        total_data_sweep = total_data_df[total_data_df['antenna_angle'] <= 180.0]
        if total_data_sweep.empty:
            total_data_sweep = total_data_df
            
        optimal_idx = total_data_sweep['total_data_MB'].idxmax() if not total_data_sweep.empty else None
        
        if optimal_idx is not None:
            opt_row = total_data_sweep.loc[optimal_idx]
            opt_angle = opt_row['antenna_angle']
            opt_data = opt_row['total_data_MB']
        else:
            opt_angle = "N/A"
            opt_data = 0.0
    else:
        opt_angle = "N/A"
        opt_data = 0.0

    # Best individual throughput
    indiv_df = df[(~df['vehicle_name'].isin(total_names)) & (df['antenna_angle'] <= 180.0)]
    if indiv_df.empty:
        indiv_df = df[~df['vehicle_name'].isin(total_names)]
        
    best_tp_idx = indiv_df['average_throughput_Gbps'].idxmax() if not indiv_df.empty else None
    if best_tp_idx is not None:
        best_tp_row = indiv_df.loc[best_tp_idx]
        best_tp_val = best_tp_row['average_throughput_Gbps']
        best_tp_vehicle = clean_label(best_tp_row['vehicle_name'])
        best_tp_angle = best_tp_row['antenna_angle']
    else:
        best_tp_val = 0.0
        best_tp_vehicle = "N/A"
        best_tp_angle = "N/A"

    # Average RSSI for all vehicles
    avg_rssi = indiv_df['average_rssi_dBm'].mean() if not indiv_df.empty else 0.0

    # Extract dynamic Y positions
    y_positions = df['y_position'].unique()
    y_pos_str = ", ".join([f"{y:.2f}m" for y in sorted(y_positions)])

    # Serialize data for Chart.js
    chart_data = {
        'angles': sorted(df['antenna_angle'].unique().tolist()),
        'vehicles': {}
    }
    
    for vehicle in df['vehicle_name'].unique():
        v_df = df[df['vehicle_name'] == vehicle].sort_values(by='antenna_angle')
        chart_data['vehicles'][vehicle] = {
            'angle': v_df['antenna_angle'].tolist(),
            'total_data_MB': v_df['total_data_MB'].tolist(),
            'connected_time_s': v_df['connected_time_s'].fillna(0).tolist(),
            'average_throughput_Gbps': v_df['average_throughput_Gbps'].fillna(0).tolist(),
            'average_rssi_dBm': v_df['average_rssi_dBm'].fillna(-100).tolist()
        }

    # Generate colors and label maps for JavaScript
    color_map = get_vehicle_colors(df)
    js_colors = {name: color_map[name] for name in df['vehicle_name'].unique()}
    js_labels = {name: clean_label(name) for name in df['vehicle_name'].unique()}

    # Use HTML template and manually replace placeholders to avoid f-string curly braces escaping issues
    html_template = """<!DOCTYPE html>
<html lang="ja">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Parameter Sweep Report - __SWEEP_ID__</title>
    <!-- Chart.js -->
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <!-- Google Fonts -->
    <link rel="preconnect" href="https://fonts.googleapis.com">
    <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
    <link href="https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;600;700&family=JetBrains+Mono:wght@400;700&display=swap" rel="stylesheet">
    <style>
        :root {
            --bg-color: #0d0e12;
            --card-bg: rgba(30, 32, 42, 0.6);
            --border-color: rgba(255, 255, 255, 0.08);
            --text-color: #f3f4f6;
            --text-muted: #9ca3af;
            --primary: #6366f1;
            --primary-hover: #4f46e5;
            --accent: #10b981;
        }

        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
        }

        body {
            font-family: 'Outfit', sans-serif;
            background-color: var(--bg-color);
            color: var(--text-color);
            line-height: 1.6;
            padding: 2rem;
            background-image: 
                radial-gradient(circle at 10% 20%, rgba(99, 102, 241, 0.05) 0%, transparent 40%),
                radial-gradient(circle at 90% 80%, rgba(233, 30, 99, 0.04) 0%, transparent 40%);
            background-attachment: fixed;
        }

        header {
            margin-bottom: 2.5rem;
            border-bottom: 1px solid var(--border-color);
            padding-bottom: 1.5rem;
            display: flex;
            justify-content: space-between;
            align-items: flex-end;
        }

        h1 {
            font-size: 2.2rem;
            font-weight: 700;
            background: linear-gradient(to right, #ffffff, #9cb3ff);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            margin-bottom: 0.5rem;
        }

        .subtitle {
            font-family: 'JetBrains Mono', monospace;
            color: var(--text-muted);
            font-size: 0.95rem;
        }

        .tag {
            background: rgba(99, 102, 241, 0.15);
            border: 1px solid rgba(99, 102, 241, 0.3);
            color: #a5b4fc;
            padding: 0.25rem 0.75rem;
            border-radius: 9999px;
            font-size: 0.8rem;
            font-weight: 600;
        }

        /* Grid Layout */
        .stats-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(240px, 1fr));
            gap: 1.5rem;
            margin-bottom: 2.5rem;
        }

        .card {
            background: var(--card-bg);
            border: 1px solid var(--border-color);
            border-radius: 16px;
            padding: 1.5rem;
            backdrop-filter: blur(12px);
            -webkit-backdrop-filter: blur(12px);
            box-shadow: 0 4px 30px rgba(0, 0, 0, 0.2);
            transition: transform 0.2s ease, border-color 0.2s ease;
        }

        .card:hover {
            transform: translateY(-2px);
            border-color: rgba(99, 102, 241, 0.2);
        }

        .card-title {
            font-size: 0.85rem;
            text-transform: uppercase;
            letter-spacing: 0.05em;
            color: var(--text-muted);
            margin-bottom: 0.5rem;
            font-weight: 600;
        }

        .card-value {
            font-size: 2rem;
            font-weight: 700;
            color: #ffffff;
            line-height: 1.2;
        }

        .card-value span {
            font-size: 1rem;
            font-weight: 400;
            color: var(--text-muted);
            margin-left: 0.25rem;
        }

        .card-desc {
            font-size: 0.8rem;
            color: var(--text-muted);
            margin-top: 0.5rem;
        }

        /* Section layout */
        .dashboard-section {
            margin-bottom: 3rem;
        }

        .section-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 1.5rem;
        }

        .section-title {
            font-size: 1.4rem;
            font-weight: 600;
            position: relative;
            padding-left: 0.75rem;
        }

        .section-title::before {
            content: '';
            position: absolute;
            left: 0;
            top: 15%;
            height: 70%;
            width: 4px;
            background: var(--primary);
            border-radius: 2px;
        }

        /* Tab buttons */
        .btn-group {
            display: flex;
            background: rgba(255, 255, 255, 0.03);
            border: 1px solid var(--border-color);
            padding: 0.25rem;
            border-radius: 8px;
        }

        .btn {
            background: transparent;
            border: none;
            color: var(--text-muted);
            padding: 0.5rem 1rem;
            font-family: inherit;
            font-size: 0.85rem;
            font-weight: 600;
            cursor: pointer;
            border-radius: 6px;
            transition: all 0.2s;
        }

        .btn.active {
            background: var(--primary);
            color: white;
            box-shadow: 0 2px 8px rgba(99, 102, 241, 0.4);
        }

        /* Charts area */
        .chart-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(500px, 1fr));
            gap: 1.5rem;
            margin-bottom: 2.5rem;
        }

        @media (max-width: 600px) {
            .chart-grid {
                grid-template-columns: 1fr;
            }
        }

        .chart-container {
            position: relative;
            height: 320px;
            width: 100%;
            margin-top: 1rem;
        }

        /* Image Gallery */
        .image-gallery {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(400px, 1fr));
            gap: 1.5rem;
        }

        .gallery-item {
            text-align: center;
        }

        .gallery-item img {
            width: 100%;
            border-radius: 12px;
            border: 1px solid var(--border-color);
            box-shadow: 0 10px 25px rgba(0,0,0,0.3);
            transition: transform 0.2s;
        }

        .gallery-item img:hover {
            transform: scale(1.02);
        }

        .gallery-title {
            font-size: 0.9rem;
            margin-top: 0.75rem;
            color: var(--text-muted);
        }

        /* Table section */
        .table-wrapper {
            overflow-x: auto;
            border-radius: 12px;
            border: 1px solid var(--border-color);
            background: var(--card-bg);
            max-height: 400px;
            overflow-y: auto;
        }

        table {
            width: 100%;
            border-collapse: collapse;
            text-align: left;
            font-family: 'JetBrains Mono', monospace;
            font-size: 0.85rem;
        }

        th {
            background: rgba(30, 32, 42, 0.9);
            position: sticky;
            top: 0;
            color: white;
            font-weight: 600;
            padding: 0.75rem 1rem;
            border-bottom: 2px solid var(--border-color);
        }

        td {
            padding: 0.75rem 1rem;
            border-bottom: 1px solid var(--border-color);
            color: var(--text-muted);
        }

        tr:hover td {
            color: white;
            background: rgba(255,255,255,0.02);
        }

        .highlight-row {
            background: rgba(99, 102, 241, 0.08);
        }

        .highlight-row td {
            color: #a5b4fc;
        }
    </style>
</head>
<body>

    <header>
        <div>
            <h1>Parameter Sweep Results Dashboard</h1>
            <div class="subtitle">Sweep ID: __SWEEP_ID__</div>
        </div>
        <div class="tag">y_position = __Y_POSITION_STR__</div>
    </header>

    <!-- Stats Cards -->
    <div class="stats-grid">
        <div class="card">
            <div class="card-title">Optimal BS Antenna Angle</div>
            <div class="card-value">__OPT_ANGLE__<span>°</span></div>
            <div class="card-desc">Angle maximizing total data transfer</div>
        </div>
        <div class="card">
            <div class="card-title">Max Total Data Transferred</div>
            <div class="card-value">__OPT_DATA__<span>MB</span></div>
            <div class="card-desc">At optimal angle of __OPT_ANGLE__°</div>
        </div>
        <div class="card">
            <div class="card-title">Peak Indiv. Throughput</div>
            <div class="card-value">__BEST_TP_VAL__<span>Gbps</span></div>
            <div class="card-desc">__BEST_TP_VEHICLE__ at __BEST_TP_ANGLE__°</div>
        </div>
        <div class="card">
            <div class="card-title">Average Sweep RSSI</div>
            <div class="card-value">__AVG_RSSI__<span>dBm</span></div>
            <div class="card-desc">Mean RSSI across all antennas</div>
        </div>
    </div>

    <!-- Interactive Charts Section -->
    <div class="dashboard-section">
        <div class="section-header">
            <h2 class="section-title">Interactive Visualizations</h2>
            <div class="btn-group">
                <button class="btn active" onclick="toggleView('interactive')">Interactive Charts</button>
                <button class="btn" onclick="toggleView('static')">Matplotlib Exports</button>
            </div>
        </div>

        <!-- Interactive Charts Grid -->
        <div id="interactive-views" class="chart-grid">
            <div class="card">
                <h3 class="card-title" style="margin-bottom:0.25rem;">Total Data Transferred (MB)</h3>
                <div class="chart-container">
                    <canvas id="chart-total-data"></canvas>
                </div>
            </div>
            <div class="card">
                <h3 class="card-title" style="margin-bottom:0.25rem;">Average Throughput (Gbps)</h3>
                <div class="chart-container">
                    <canvas id="chart-throughput"></canvas>
                </div>
            </div>
            <div class="card">
                <h3 class="card-title" style="margin-bottom:0.25rem;">Average RSSI (dBm)</h3>
                <div class="chart-container">
                    <canvas id="chart-rssi"></canvas>
                </div>
            </div>
            <div class="card">
                <h3 class="card-title" style="margin-bottom:0.25rem;">Connected Time (seconds)</h3>
                <div class="chart-container">
                    <canvas id="chart-connected-time"></canvas>
                </div>
            </div>
        </div>

        <!-- Static Image Grid (initially hidden) -->
        <div id="static-views" class="image-gallery" style="display: none;">
            <div class="gallery-item">
                <img src="plots/total_data.png" alt="Total Data Transferred">
                <div class="gallery-title">Total Data Transferred (MB)</div>
            </div>
            <div class="gallery-item">
                <img src="plots/throughput.png" alt="Average Throughput">
                <div class="gallery-title">Average Throughput (Gbps)</div>
            </div>
            <div class="gallery-item">
                <img src="plots/rssi.png" alt="Average RSSI">
                <div class="gallery-title">Average RSSI (dBm)</div>
            </div>
            <div class="gallery-item">
                <img src="plots/connected_time.png" alt="Connected Time">
                <div class="gallery-title">Connected Time (seconds)</div>
            </div>
        </div>
    </div>

    <!-- Data Table Section -->
    <div class="dashboard-section">
        <h2 class="section-title" style="margin-bottom:1.5rem;">Sweep Summary Raw Data</h2>
        <div class="table-wrapper">
            <table>
                <thead>
                    <tr>
                        <th>Antenna Angle (°)</th>
                        <th>Vehicle / Node</th>
                        <th>Total Data (MB)</th>
                        <th>Connected Time (s)</th>
                        <th>Avg Throughput (Gbps)</th>
                        <th>Avg RSSI (dBm)</th>
                        <th>Handovers</th>
                    </tr>
                </thead>
                <tbody>
                    __TABLE_ROWS__
                </tbody>
            </table>
        </div>
    </div>

    <script>
        // Data injected from Python
        const rawData = __CHART_DATA__;
        const colors = __JS_COLORS__;
        const labelMap = __JS_LABELS__;
        const totals = __JS_TOTALS__;
        
        function toggleView(mode) {
            const interactive = document.getElementById('interactive-views');
            const staticViews = document.getElementById('static-views');
            const buttons = document.querySelectorAll('.btn-group .btn');
            
            if (mode === 'interactive') {
                interactive.style.display = 'grid';
                staticViews.style.display = 'none';
                buttons[0].classList.add('active');
                buttons[1].classList.remove('active');
            } else {
                interactive.style.display = 'none';
                staticViews.style.display = 'grid';
                buttons[0].classList.remove('active');
                buttons[1].classList.add('active');
            }
        }

        // Setup Chart.js Defaults
        Chart.defaults.color = '#9ca3af';
        Chart.defaults.borderColor = 'rgba(255, 255, 255, 0.06)';
        Chart.defaults.font.family = "'Outfit', sans-serif";

        const angles = rawData.angles.filter(a => a <= 180.0); // Filter for line rendering

        // Common Chart Config options
        const commonOptions = (yTitle) => ({
            responsive: true,
            maintainAspectRatio: false,
            interaction: {
                mode: 'index',
                intersect: false
            },
            plugins: {
                legend: {
                    position: 'top',
                    labels: {
                        boxWidth: 12,
                        usePointStyle: true,
                        pointStyle: 'circle'
                    }
                }
            },
            scales: {
                x: {
                    title: {
                        display: true,
                        text: 'Antenna Angle (deg)',
                        color: '#f3f4f6'
                    },
                    grid: {
                        display: false
                    }
                },
                y: {
                    title: {
                        display: true,
                        text: yTitle,
                        color: '#f3f4f6'
                    }
                }
            }
        });

        // Helper to extract dataset
        function getDatasets(metric, excludeTotal = false) {
            const ds = [];
            for (const [vName, vData] of Object.entries(rawData.vehicles)) {
                if (excludeTotal && totals.includes(vName)) continue;
                
                // Map the data to match filtered angles
                const mappedData = angles.map(a => {
                    const idx = vData.angle.indexOf(a);
                    return idx !== -1 ? vData[metric][idx] : null;
                });

                const isTotal = totals.includes(vName);
                ds.push({
                    label: labelMap[vName] || vName,
                    data: mappedData,
                    borderColor: colors[vName] || '#fff',
                    backgroundColor: isTotal ? 'rgba(99, 102, 241, 0.08)' : 'transparent',
                    borderWidth: isTotal ? 3 : 2,
                    tension: 0.2,
                    fill: isTotal
                });
            }
            return ds;
        }

        // Render Total Data Chart
        new Chart(document.getElementById('chart-total-data'), {
            type: 'line',
            data: {
                labels: angles,
                datasets: getDatasets('total_data_MB')
            },
            options: commonOptions('Data Transferred (MB)')
        });

        // Render Throughput Chart
        new Chart(document.getElementById('chart-throughput'), {
            type: 'line',
            data: {
                labels: angles,
                datasets: getDatasets('average_throughput_Gbps')
            },
            options: commonOptions('Throughput (Gbps)')
        });

        // Render RSSI Chart
        new Chart(document.getElementById('chart-rssi'), {
            type: 'line',
            data: {
                labels: angles,
                datasets: getDatasets('average_rssi_dBm')
            },
            options: {
                ...commonOptions('RSSI (dBm)'),
                scales: {
                    x: {
                        title: {
                            display: true,
                            text: 'Antenna Angle (deg)',
                            color: '#f3f4f6'
                        },
                        grid: {
                            display: false
                        }
                    },
                    y: {
                        title: {
                            display: true,
                            text: 'RSSI (dBm)',
                            color: '#f3f4f6'
                        },
                        suggestedMin: -80,
                        suggestedMax: -45
                    }
                }
            }
        });

        // Render Connected Time Chart
        new Chart(document.getElementById('chart-connected-time'), {
            type: 'line',
            data: {
                labels: angles,
                datasets: getDatasets('connected_time_s')
            },
            options: commonOptions('Connected Time (seconds)')
        });

    </script>
</body>
</html>
"""
    # Replace the placeholders
    html_content = html_template.replace('__SWEEP_ID__', str(sweep_id))
    html_content = html_content.replace('__Y_POSITION_STR__', y_pos_str)
    html_content = html_content.replace('__OPT_ANGLE__', f"{opt_angle:.1f}" if isinstance(opt_angle, float) else str(opt_angle))
    html_content = html_content.replace('__OPT_DATA__', f"{opt_data:.2f}")
    html_content = html_content.replace('__BEST_TP_VAL__', f"{best_tp_val:.2f}")
    html_content = html_content.replace('__BEST_TP_VEHICLE__', str(best_tp_vehicle))
    html_content = html_content.replace('__BEST_TP_ANGLE__', f"{best_tp_angle:.1f}" if isinstance(best_tp_angle, float) else str(best_tp_angle))
    html_content = html_content.replace('__AVG_RSSI__', f"{avg_rssi:.1f}")
    html_content = html_content.replace('__CHART_DATA__', json.dumps(chart_data))
    html_content = html_content.replace('__JS_COLORS__', json.dumps(js_colors))
    html_content = html_content.replace('__JS_LABELS__', json.dumps(js_labels))
    html_content = html_content.replace('__JS_TOTALS__', json.dumps(total_names))
    html_content = html_content.replace('__TABLE_ROWS__', generate_table_rows(df, opt_angle))

    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(html_content)

def generate_table_rows(df, opt_angle):
    """Generate HTML table rows sorted by angle and vehicle name"""
    df_sorted = df.sort_values(by=['antenna_angle', 'vehicle_name'])
    rows = []

    for _, r in df_sorted.iterrows():
        angle = r['antenna_angle']
        vName = r['vehicle_name']
        vPretty = clean_label(vName)
        
        is_opt = (isinstance(opt_angle, float) and abs(angle - opt_angle) < 0.01)
        row_cls = ' class="highlight-row"' if is_opt else ''
        
        # Safe float formatting
        tot_data = f"{r['total_data_MB']:.3f}" if not pd.isna(r['total_data_MB']) else '-'
        conn_time = f"{r['connected_time_s']:.3f}" if not pd.isna(r['connected_time_s']) else '-'
        thru = f"{r['average_throughput_Gbps']:.3f}" if not pd.isna(r['average_throughput_Gbps']) else '-'
        rssi = f"{r['average_rssi_dBm']:.3f}" if not pd.isna(r['average_rssi_dBm']) else '-'
        handovers = f"{r['handover_count']:.1f}" if not pd.isna(r['handover_count']) else '-'
        
        rows.append(f"""                    <tr{row_cls}>
            <td>{angle}°</td>
            <td>{vPretty}</td>
            <td>{tot_data}</td>
            <td>{conn_time}</td>
            <td>{thru}</td>
            <td>{rssi}</td>
            <td>{handovers}</td>
        </tr>""")
                    
    return "\n".join(rows)

def generate_generic_plots(df, plots_dir, x_var, series_var=None, vehicle=None):
    """任意のスイープ変数列を x 軸にした汎用プロット群を生成する。

    x_var:      x軸に使う列名 (例 "density")
    series_var: 系列の分割に使う列名 (例 "method"、省略時は vehicle_name 系列)
    vehicle:    対象 vehicle_name (省略時は *_total 行、無ければ全行)
    """
    os.makedirs(plots_dir, exist_ok=True)
    setup_matplotlib_style()

    for col in [x_var] + ([series_var] if series_var else []):
        if col not in df.columns:
            print(f"Error: column '{col}' not in sweep_summary.csv "
                  f"(available: {', '.join(df.columns)})")
            sys.exit(1)

    if vehicle:
        data = df[df['vehicle_name'] == vehicle]
    else:
        totals = df[df['vehicle_name'].astype(str).str.endswith('_total')]
        data = totals if not totals.empty else df

    metrics = [
        ('total_data_MB', 'Total transferred data [MB]'),
        ('average_throughput_Gbps', 'Average throughput [Gbps]'),
        ('average_rssi_dBm', 'Average RSSI [dBm]'),
        ('connected_time_s', 'Connected time [s]'),
        ('handover_count', 'Handover count'),
    ]
    series_col = series_var if series_var else 'vehicle_name'
    numeric_x = pd.api.types.is_numeric_dtype(data[x_var])

    for metric, ylabel in metrics:
        if metric not in data.columns:
            continue
        fig, ax = plt.subplots(figsize=(6.4, 4.2))
        for key, grp in data.groupby(series_col):
            grp = grp.sort_values(x_var)
            x = grp[x_var] if numeric_x else grp[x_var].astype(str)
            ax.errorbar(x, grp[metric],
                        yerr=grp.get(f'{metric}_std'),
                        label=str(key), marker='o', markersize=4.5,
                        linewidth=1.8, capsize=3)
        ax.set_xlabel(x_var)
        ax.set_ylabel(ylabel)
        if numeric_x:
            ax.set_xticks(sorted(data[x_var].unique()))
        ax.grid(alpha=0.3, linewidth=0.5)
        ax.legend(fontsize=8, title=series_col)
        fig.tight_layout()
        out = os.path.join(plots_dir, f"{metric}_vs_{x_var}.png")
        fig.savefig(out, dpi=300)
        plt.close(fig)
        print(f"  wrote {out}")


def main():
    parser = argparse.ArgumentParser(description="Generate plots and HTML report for sweep simulation")
    parser.add_argument("sweep_dir", nargs="?", default=None, help="Path to sweep results directory (e.g. sim_results/sweep_20260605_160503)")
    parser.add_argument("--x-var", default=None,
                        help="x軸に使う列名 (例: density)。指定時は汎用プロットモード")
    parser.add_argument("--series-var", default=None,
                        help="系列分割に使う列名 (例: method)。--x-var と併用")
    parser.add_argument("--vehicle", default=None,
                        help="汎用モードで対象にする vehicle_name (省略時 *_total)")
    args = parser.parse_args()
    
    sweep_dir = args.sweep_dir
    if sweep_dir is None:
        # Auto-detect latest sweep directory
        sim_results_dir = "sim_results"
        if not os.path.exists(sim_results_dir):
            print(f"Error: {sim_results_dir} directory does not exist.")
            sys.exit(1)
            
        # sweep_summary.csv を持つディレクトリを対象 (output_name でリネームされた結果も含む)
        sweeps = [os.path.join(sim_results_dir, d) for d in os.listdir(sim_results_dir)
                  if os.path.exists(os.path.join(sim_results_dir, d, "sweep_summary.csv"))]
        if not sweeps:
            print("Error: No sweep directories found in sim_results.")
            sys.exit(1)
            
        # Get latest sweep
        sweep_dir = max(sweeps, key=os.path.getmtime)
        print(f"No sweep directory specified. Auto-detected latest: {sweep_dir}")
        
    summary_path = os.path.join(sweep_dir, "sweep_summary.csv")
    if not os.path.exists(summary_path):
        print(f"Error: {summary_path} does not exist.")
        sys.exit(1)
        
    print(f"Reading data from {summary_path}...")
    df = pd.read_csv(summary_path)

    if args.x_var:
        plots_dir = os.path.join(sweep_dir, "plots")
        print(f"Generating generic plots (x={args.x_var}, series={args.series_var})...")
        generate_generic_plots(df, plots_dir, args.x_var, args.series_var, args.vehicle)
        print(f"Generic plots saved to {plots_dir}")
        return

    plots_dir = os.path.join(sweep_dir, "plots")
    print("Generating static matplotlib plots...")
    generate_static_plots(df, plots_dir)
    print(f"Static plots saved to {plots_dir}")
    
    sweep_id = os.path.basename(sweep_dir)
    report_path = os.path.join(sweep_dir, "index.html")
    print("Generating interactive HTML report...")
    generate_html_report(df, report_path, sweep_id)
    print(f"HTML report saved to {report_path}")
    
    print("\nPlotting complete! You can open the HTML report or view the static plots in:")
    print(f"  - Report: {os.path.abspath(report_path)}")
    print(f"  - Static plots: {os.path.abspath(plots_dir)}/")

if __name__ == "__main__":
    main()

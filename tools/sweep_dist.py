#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import json
import glob
import math
import shutil
import argparse
import datetime
import re

# Add script directory to python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from lib.sweep_config import (
    Y_POSITIONS,
    RX_YAW_DEG,
    START_ANGLE,
    END_ANGLE,
    STEP_ANGLE,
    NUM_RUNS,
    TASK_TIMEOUT_SEC,
    SWEEP_REAL_TIME_FACTOR,
    ANGLES_DEG
)
from lib.sweep_data import average_summaries

def fix_ownership(sweep_dir):
    is_docker = os.path.exists('/.dockerenv')
    import subprocess
    try:
        if is_docker:
            # Container side: read host user's UID/GID from /workspace mount
            if os.path.exists('/workspace'):
                stat_info = os.stat('/workspace')
                uid = stat_info.st_uid
                gid = stat_info.st_gid
                if os.path.exists(sweep_dir):
                    subprocess.run(
                        ["chown", "-R", f"{uid}:{gid}", sweep_dir],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        timeout=10
                    )
        else:
            # Host side
            uid = os.getuid()
            gid = os.getgid()
            container_dir = sweep_dir
            if not sweep_dir.startswith('/workspace'):
                container_dir = os.path.join('/workspace', sweep_dir)
            print(f"[Sweep Dist] Fixing ownership of {container_dir} to {uid}:{gid}...")
            subprocess.run(
                ["docker", "compose", "exec", "-T", "sim", "chown", "-R", f"{uid}:{gid}", container_dir],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10
            )
    except Exception as e:
        print(f"[Sweep Dist] Warning: Failed to fix ownership: {e}")

def handle_split(args):
    num_splits = args.splits
    if num_splits <= 0:
        print("Error: Number of splits must be greater than 0.")
        sys.exit(1)
        
    sweep_id = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    dist_dir = os.path.join("sim_results", f"dist_sweep_{sweep_id}")
    os.makedirs(dist_dir, exist_ok=True)
    
    # Generate complete list of tasks
    tasks = []
    overall_task_no = 1
    for run_idx in range(1, NUM_RUNS + 1):
        for y in Y_POSITIONS:
            for angle_deg in ANGLES_DEG:
                tasks.append({
                    "run_idx": run_idx,
                    "y": y,
                    "angle_deg": angle_deg,
                    "overall_task_no": overall_task_no
                })
                overall_task_no += 1
                
    total_tasks = len(tasks)
    chunk_size = math.ceil(total_tasks / num_splits)
    
    print(f"Splitting {total_tasks} tasks (NUM_RUNS={NUM_RUNS}, Y_POSITIONS={Y_POSITIONS}, angles={len(ANGLES_DEG)}) into {num_splits} chunks...")
    print(f"Target chunk size: ~{chunk_size} tasks per chunk.")
    
    # Save config dict to embed in each manifest
    config_dict = {
        "Y_POSITIONS": Y_POSITIONS,
        "RX_YAW_DEG": RX_YAW_DEG,
        "START_ANGLE": START_ANGLE,
        "END_ANGLE": END_ANGLE,
        "STEP_ANGLE": STEP_ANGLE,
        "NUM_RUNS": NUM_RUNS,
        "TASK_TIMEOUT_SEC": TASK_TIMEOUT_SEC,
        "SWEEP_REAL_TIME_FACTOR": SWEEP_REAL_TIME_FACTOR,
        "ANGLES_DEG": ANGLES_DEG
    }
    
    manifest_paths = []
    for i in range(num_splits):
        start_idx = i * chunk_size
        end_idx = min(start_idx + chunk_size, total_tasks)
        chunk_tasks = tasks[start_idx:end_idx] if start_idx < total_tasks else []
        
        manifest = {
            "sweep_id": sweep_id,
            "chunk_idx": i,
            "total_chunks": num_splits,
            "config": config_dict,
            "tasks": chunk_tasks
        }
        
        manifest_path = os.path.join(dist_dir, f"manifest_chunk_{i}.json")
        with open(manifest_path, 'w', encoding='utf-8') as f:
            json.dump(manifest, f, indent=2)
        manifest_paths.append(manifest_path)
        
    # Write instructions.txt
    instr_path = os.path.join(dist_dir, "instructions.txt")
    with open(instr_path, 'w', encoding='utf-8') as f:
        f.write("=== Distributed Parameter Sweep Instructions ===\n\n")
        f.write(f"Sweep ID: {sweep_id}\n")
        f.write(f"Total Chunks: {num_splits}\n")
        f.write(f"Total Tasks: {total_tasks}\n\n")
        f.write("Execution Steps:\n")
        f.write("1. Copy the manifest_chunk_X.json files to each PC.\n")
        f.write("2. On each PC, run the sweep simulation by specifying its chunk manifest:\n")
        f.write("   python3 tools/sweep_sim.py --manifest path/to/manifest_chunk_X.json\n")
        f.write("3. After runs finish, gather the results directory 'sim_results/sweep_{sweep_id}' from all PCs.\n")
        f.write("4. Merge all files into the same directory on the primary PC (no files will conflict/overwrite).\n")
        f.write("5. Run the merge tool on the primary PC:\n")
        f.write(f"   python3 tools/sweep_dist.py merge --sweep-dir sim_results/sweep_{sweep_id}\n\n")
        
    fix_ownership(dist_dir)
    
    print(f"\nSuccessfully created {num_splits} chunks in: {dist_dir}")
    print(f"Instructions written to: {instr_path}")
    print("\nTo run a chunk:")
    for i in range(num_splits):
        print(f"  PC {i+1}: python3 tools/sweep_sim.py --manifest {dist_dir}/manifest_chunk_{i}.json")

def handle_merge(args):
    sweep_dir = args.sweep_dir
    if not sweep_dir:
        print("Error: Please specify the --sweep-dir path.")
        sys.exit(1)
        
    if not os.path.exists(sweep_dir):
        print(f"Error: Sweep directory {sweep_dir} does not exist.")
        sys.exit(1)
        
    # Find all manifest files in the directory
    manifests = glob.glob(os.path.join(sweep_dir, "manifest_chunk_*.json"))
    if not manifests:
        print(f"Error: No manifest_chunk_*.json files found in {sweep_dir}.")
        print("Please make sure you copied the manifest files along with the results.")
        sys.exit(1)
        
    # Read one manifest to get total_chunks and configurations
    with open(manifests[0], 'r', encoding='utf-8') as f:
        meta = json.load(f)
    
    total_chunks = meta.get('total_chunks')
    sweep_id = meta.get('sweep_id')
    num_runs = meta.get('config', {}).get('NUM_RUNS', NUM_RUNS)
    
    # Check if all chunks are present
    present_chunk_indices = []
    for m in manifests:
        match = re.search(r'manifest_chunk_(\d+)\.json', m)
        if match:
            present_chunk_indices.append(int(match.group(1)))
            
    present_chunk_indices = sorted(list(set(present_chunk_indices)))
    missing_chunks = [i for i in range(total_chunks) if i not in present_chunk_indices]
    
    if missing_chunks:
        print(f"Warning: Only found chunks {present_chunk_indices} out of {total_chunks}.")
        print(f"Missing chunks: {missing_chunks}")
        if not args.force:
            ans = input("Do you want to proceed with merging the available data? (y/n): ")
            if ans.strip().lower() != 'y':
                print("Merge cancelled.")
                sys.exit(0)
                
    # Now merge chunk CSVs for each run
    print(f"\nMerging chunk files for sweep {sweep_id}...")
    for run_idx in range(1, num_runs + 1):
        merged_summary_file = os.path.join(sweep_dir, f"sweep_summary_run{run_idx}.csv")
        
        # Find all chunk summary files for this run
        chunk_csv_pattern = os.path.join(sweep_dir, f"sweep_summary_run{run_idx}_chunk*.csv")
        chunk_csvs = glob.glob(chunk_csv_pattern)
        
        if not chunk_csvs:
            continue
            
        merged_rows = []
        header = None
        
        # If there's an existing merged file, we load it (to allow incremental merges)
        if os.path.exists(merged_summary_file):
            try:
                with open(merged_summary_file, 'r', encoding='utf-8') as f:
                    lines = f.readlines()
                    if lines:
                        header = lines[0]
                        merged_rows.extend(lines[1:])
            except Exception:
                pass
                
        for csv_file in sorted(chunk_csvs):
            try:
                with open(csv_file, 'r', encoding='utf-8') as infile:
                    lines = infile.readlines()
                    if lines:
                        if not header:
                            header = lines[0]
                        merged_rows.extend(lines[1:])
            except Exception as e:
                print(f"Warning: Failed to read {csv_file}: {e}")
                
        # Write merged run file
        if header and merged_rows:
            with open(merged_summary_file, 'w', encoding='utf-8') as outfile:
                outfile.write(header)
                outfile.writelines(merged_rows)
            print(f"  Merged Run {run_idx} CSV.")
            
        # Clean up chunk CSVs
        for csv_file in chunk_csvs:
            try:
                os.remove(csv_file)
            except Exception as e:
                print(f"Warning: Failed to delete {csv_file}: {e}")
                
    # Call average_summaries to generate the final averaged CSV
    final_summary_file = os.path.join(sweep_dir, "sweep_summary.csv")
    print("\nGenerating averaged summaries...")
    try:
        summary_files = [os.path.join(sweep_dir, f"sweep_summary_run{run_idx}.csv") for run_idx in range(1, num_runs + 1)]
        # Filter files that actually exist
        summary_files = [f for f in summary_files if os.path.exists(f)]
        if summary_files:
            average_summaries(summary_files, final_summary_file)
            print(f"Successfully generated averaged summary: {final_summary_file}")
        else:
            print("Error: No run summary files were generated. Cannot average.")
    except Exception as e:
        print(f"Error during averaging: {e}")
        
    fix_ownership(sweep_dir)
    print("\nMerge completed successfully!")

def main():
    parser = argparse.ArgumentParser(description="Distributed Parameter Sweep Utility")
    subparsers = parser.add_subparsers(dest="command", required=True)
    
    # Split subcommand
    split_parser = subparsers.add_parser("split", help="Split parameter sweep tasks into manifests")
    split_parser.add_argument("--splits", "-n", type=int, required=True, help="Number of chunks to split the sweep into")
    
    # Merge subcommand
    merge_parser = subparsers.add_parser("merge", help="Merge split chunk results")
    merge_parser.add_argument("--sweep-dir", "-d", type=str, required=True, help="Path to the sweep results directory to merge")
    merge_parser.add_argument("--force", "-f", action="store_true", help="Force merge even if some chunks are missing")
    
    args = parser.parse_args()
    
    if args.command == "split":
        handle_split(args)
    elif args.command == "merge":
        handle_merge(args)

if __name__ == "__main__":
    main()

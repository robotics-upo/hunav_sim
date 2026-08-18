#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os

def main():
    parser = argparse.ArgumentParser(description="Process hunav evaluation results.")
    parser.add_argument('files', nargs='+', help='List of base CSV files')
    args = parser.parse_args()

    # Data structure to hold the processed base metrics for comparison table
    # Columns will be the file names or experiment tags
    comparison_data = {}
    
    # We will also keep track of averaged steps DataFrames for plotting
    # Dict mapping filename -> Dict of experiment_tag -> averaged DataFrame
    steps_data_dict = {}

    results_dir = "/home/hunav_gz_classic_ws/src/hunav_sim/hunav_evaluator/results"
    for arg_file in args.files:
        filename = os.path.basename(arg_file)
        filepath = os.path.join(results_dir, filename)

        if not os.path.exists(filepath):
            print(f"File not found: {filepath}")
            continue

        base_dir = results_dir

        # Read base CSV
        df = pd.read_csv(filepath)
        
        # Base CSV has 'experiment_tag' as the first column usually.
        # Check if the columns exist
        if 'experiment_tag' not in df.columns or 'run_id' not in df.columns:
            print(f"Required columns 'experiment_tag' and 'run_id' not found in {filepath}")
            continue

        # Group by experiment_tag
        grouped = df.groupby('experiment_tag')
        
        for exp_tag, group in grouped:
            numeric_cols = group.select_dtypes(include=[np.number]).drop(columns=['run_id'], errors='ignore')
            
            # Compute average and std for base metrics
            avg_metrics = numeric_cols.mean()
            std_metrics = numeric_cols.std()
            
            # Combine into a single string "mean ± std" if there are multiple runs, 
            # otherwise just use the mean.
            num_runs = len(group)
            col_name = f"{filename}_{exp_tag}"
            comparison_data[col_name] = {}
            
            for metric in numeric_cols.columns:
                if num_runs > 1:
                    comparison_data[col_name][metric] = f"{avg_metrics[metric]:.4f} ± {std_metrics[metric]:.4f}"
                else:
                    comparison_data[col_name][metric] = f"{avg_metrics[metric]:.4f}"
                    
            # Process corresponding steps files
            steps_dfs = []
            for run_id in group['run_id']:
                step_filename = filepath.replace(".csv", f"_steps_{exp_tag}_{run_id}.csv")
                if os.path.exists(step_filename):
                    df_step = pd.read_csv(step_filename)
                    if 'time_stamps' in df_step.columns:
                        df_step.set_index('time_stamps', inplace=True)
                    steps_dfs.append(df_step)
                else:
                    print(f"Warning: Step file not found: {step_filename}")
            
            if steps_dfs:
                # To average time series, we concatenate and group by index (time_stamps)
                combined_steps = pd.concat(steps_dfs)
                # Round index to 2 decimal places to align timestamps that might be slightly off
                rounded_index = np.round(combined_steps.index, decimals=2)
                combined_steps.index = rounded_index
                avg_steps = combined_steps.groupby(combined_steps.index).mean()
                
                if filename not in steps_data_dict:
                    steps_data_dict[filename] = {}
                steps_data_dict[filename][exp_tag] = avg_steps

    # Generate comparison table
    if comparison_data:
        comp_df = pd.DataFrame(comparison_data)
        print("\n--- Comparison Table ---")
        print(comp_df)
        
        # Save comparison table in the results directory
        out_dir = "/home/hunav_gz_classic_ws/src/hunav_sim/hunav_evaluator/results"
        comp_csv = os.path.join(out_dir, "comparison_table.csv")
        comp_df.to_csv(comp_csv)
        print(f"Comparison table saved to: {comp_csv}")
    
    # Plot metrics
    # Collect all unique metrics from steps data
    all_metrics = set()
    for file_data in steps_data_dict.values():
        for exp_data in file_data.values():
            all_metrics.update(exp_data.columns)

    if all_metrics:
        out_dir = "/home/hunav_gz_classic_ws/src/hunav_sim/hunav_evaluator/results"
        
        for metric in all_metrics:
            plt.figure(figsize=(10, 6))
            
            plotted_something = False
            for filename, file_data in steps_data_dict.items():
                for exp_tag, df_step in file_data.items():
                    if metric in df_step.columns:
                        # Drop NA values for plotting
                        plot_data = df_step[metric].dropna()
                        if not plot_data.empty:
                            label = f"{filename} ({exp_tag})"
                            plt.plot(plot_data.index, plot_data.values, label=label)
                            plotted_something = True
            
            if plotted_something:
                plt.title(f"Time-series comparison: {metric}")
                plt.xlabel("Time (s)")
                plt.ylabel(metric)
                plt.legend()
                plt.grid(True)
                
                plot_filename = os.path.join(out_dir, f"plot_{metric}.png")
                plt.savefig(plot_filename)
                print(f"Saved plot: {plot_filename}")
            plt.close()

if __name__ == "__main__":
    main()

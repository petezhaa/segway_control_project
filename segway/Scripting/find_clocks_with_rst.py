import os
import re
from pathlib import Path
from datetime import datetime

def find_testbenches_with_clk_rst(root_dir):
    """
    Find all SystemVerilog files that have clock signals WITHOUT reset signals.
    Looks for patterns like .clk(clk) but NO .rst_n(rst_n) or .rst(...).
    Searches all .sv files in the project, not just testbenches.
    """
    results = {
        'files_with_clk_no_rst': [],
        'files_with_clk_and_rst': [],
        'all_sv_files': []
    }
    
    # Pattern to match .clk(...) and .rst_n(...) or .rst(...)
    clk_pattern = r'\.clk\s*\(\s*\w+\s*\)'
    rst_pattern = r'\.rst[_n]*\s*\(\s*\w+\s*\)'
    
    root_path = Path(root_dir)
    
    # Find all .sv files recursively
    for sv_file in root_path.glob('**/*.sv'):
        # Skip work directories
        if 'work' in sv_file.parts or '.git' in sv_file.parts:
            continue
            
        with open(sv_file, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
        
        relative_path = str(sv_file.relative_to(root_dir))
        results['all_sv_files'].append(relative_path)
        
        has_clk = bool(re.search(clk_pattern, content))
        has_rst = bool(re.search(rst_pattern, content))
        
        if has_clk and not has_rst:
            results['files_with_clk_no_rst'].append({
                'file': relative_path,
                'path': str(sv_file)
            })
        elif has_clk and has_rst:
            results['files_with_clk_and_rst'].append({
                'file': relative_path,
                'path': str(sv_file)
            })
    
    return results

def print_results(results):
    """Print the results in a professional report format."""
    timestamp = datetime.now().strftime("%m/%d/%Y %I:%M %p")
    
    print("\n")
    print("=" * 85)
    print(" " * 20 + "SEGWAY DESIGN VALIDATION REPORT")
    print(" " * 15 + "Clock and Reset Signal Analysis")
    print("=" * 85)
    print(f"Last updated {timestamp}")
    print("=" * 85)
    
    print("\nOVERVIEW")
    print("-" * 85)
    print(f"Total SystemVerilog files scanned: {len(results['all_sv_files'])}")
    print(f"Files with clock but missing reset: {len(results['files_with_clk_no_rst'])}")
    print(f"Files with proper clock and reset: {len(results['files_with_clk_and_rst'])}")
    print()
    
    if results['files_with_clk_no_rst']:
        print("\nCRITICAL ISSUES - Missing Reset Signals")
        print("-" * 85)
        print("\nThe following files have clock signals but are missing reset signals.")
        print("These modules need reset signals for proper post-synthesis simulation.\n")
        
        for i, item in enumerate(results['files_with_clk_no_rst'], 1):
            print(f"{i}. {item['file']}")
            print(f"   Path: {item['path']}\n")
    else:
        print("\nSTATUS: OK")
        print("-" * 85)
        print("All files with clock signals have corresponding reset signals.")
        print()
    
    if results['files_with_clk_and_rst']:
        print("\nVERIFIED FILES - Clock and Reset Present")
        print("-" * 85)
        print(f"Total verified files: {len(results['files_with_clk_and_rst'])}\n")
        
        for i, item in enumerate(results['files_with_clk_and_rst'], 1):
            print(f"{i}. {item['file']}")
    
    print("\n" + "=" * 85)
    print("\nRECOMMENDATIONS")
    print("-" * 85)
    
    if results['files_with_clk_no_rst']:
        print("\n1. ADD RESET SIGNALS")
        print("   For each file with missing reset signals, add a reset input parameter")
        print("   and ensure it connects to all synchronous logic elements.")
        print()
        print("2. TESTBENCH UPDATES")
        print("   Update testbenches to include reset signal timing:")
        print("   - Assert reset at time 0")
        print("   - Wait for positive clock edge")
        print("   - Wait for negative clock edge")
        print("   - Deassert reset on negative clock edge")
        print()
        print("3. RESET TIMING REQUIREMENTS")
        print("   - Reset pulse must cover BOTH positive and negative clock edges")
        print("   - Deassert reset on negative edge of clock")
        print("   - All flip-flops must be initialized before reset is deasserted")
    else:
        print("No critical issues detected. All clock signals have reset signals.")
    
    print("\n" + "=" * 85 + "\n")
    
    print("\n" + "=" * 80)

if __name__ == '__main__':
    root_dir = r'C:\Users\peter\Documents\projects\segway'
    results = find_testbenches_with_clk_rst(root_dir)
    print_results(results)

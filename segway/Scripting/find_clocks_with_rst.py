import os
import re
from pathlib import Path

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
    """Print the results in a formatted way."""
    print("=" * 80)
    print("ALL SYSTEMVERILOG FILES ANALYSIS - Clock WITHOUT Reset Signals")
    print("=" * 80)
    
    print(f"\nTotal .sv files found: {len(results['all_sv_files'])}")
    print(f"Files with CLK but NO RST: {len(results['files_with_clk_no_rst'])}")
    print(f"Files with CLK and RST: {len(results['files_with_clk_and_rst'])}")
    
    print("\n" + "=" * 80)
    print("FILES WITH CLOCK BUT NO RESET")
    print("=" * 80)
    if results['files_with_clk_no_rst']:
        for item in results['files_with_clk_no_rst']:
            print(f"  [MISSING] {item['file']}")
    else:
        print("  None found - all files with clocks have resets!")
    
    if results['files_with_clk_and_rst']:
        print("\n" + "=" * 80)
        print("FILES WITH CLOCK AND RESET (OK)")
        print("=" * 80)
        for item in results['files_with_clk_and_rst']:
            print(f"  [OK] {item['file']}")
    
    print("\n" + "=" * 80)

if __name__ == '__main__':
    root_dir = r'C:\Users\peter\Documents\projects\segway'
    results = find_testbenches_with_clk_rst(root_dir)
    print_results(results)

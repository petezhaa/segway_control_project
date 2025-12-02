#!/usr/bin/env python3
"""
Script to automatically compile and run all SystemVerilog testbenches
for the Segway project using ModelSim/QuestaSim.
"""

import subprocess
import sys
import os
from pathlib import Path
from datetime import datetime
import json

# Configuration
WORK_LIB = "work"
# ModelSim/QuestaSim flags:
# -c: command line mode (no GUI)
# -voptargs=+acc: enable access for debugging/coverage
# -sv_seed random: randomize seed for SystemVerilog randomization
VSIM_FLAGS = "-c -voptargs=+acc -sv_seed random"

# Source files are in the parent directory
SOURCE_DIR = ".."

COMPILE_ORDER = [
    # Support files and packages first
    "../task_pkg.sv",
    "../over_I_task_pkg.sv",
    
    # Design files (modules used by testbenches)
    "../UART_tx.sv",
    "../UART_rx.sv",
    "../PWM11.sv",
    "../SPI_mnrch.sv",
    "../SPI_ADC128S.sv",
    "../A2D_intf.sv",
    "../inert_intf.sv",
    "../rst_synch.sv",
    "../PID.sv",
    "../mtr_drv.sv",
    "../balance_cntrl.sv",
    "../steer_en_SM.sv",
    "../steer_en.sv",
    "../Auth_blk.sv",
    "../piezo_drv.sv",
    "../Segway.sv",
    
    # Model/testbench support files
    "../inertial_integrator.sv",
    "../SegwayMath.sv",
    "../SegwayModel.sv",
    "../ADC128S_FC.sv",
]

# Testbenches organized by directory
# Each entry has: "module" (the actual module name in the .sv file) and "path" (file location)
TESTBENCH_SETS = {
    "tests": [
        {"module": "Auth_segway_tb", "path": "../tests/Auth_segway_tb.sv"},
        {"module": "linear_speed_tb", "path": "../tests/linear_speed_tb.sv"},
        {"module": "over_I_tb", "path": "../tests/over_I_tb.sv"},
        {"module": "Segway_tb", "path": "../tests/Segway_tb.sv"},
        {"module": "steering_response_tb", "path": "../tests/steering_response_tb.sv"},
        {"module": "simultaneous_faults_tb", "path": "../tests/simultaneous_faults_tb.sv"},
    ],
    "physics_tests": [
        {"module": "Auth_segway_tb", "path": "../physics_tests/Auth_segway_phys_tb.sv"},
        {"module": "Auth_segway_tb", "path": "../physics_tests/Auth_segway_tb.sv"},
        {"module": "linear_speed_tb", "path": "../physics_tests/linear_speed_phys_tb.sv"},
        {"module": "over_I_tb", "path": "../physics_tests/over_I_phys_tb.sv"},
        {"module": "Segway_tb", "path": "../physics_tests/Segway_phys_tb.sv"},
        {"module": "steering_response_tb", "path": "../physics_tests/steering_response_phys_tb.sv"},
    ],
    "ansley": [
        {"module": "Auth_segway_tb", "path": "../Ansley phys TB/Auth_segway_tb.sv"},
        {"module": "linear_speed_tb", "path": "../Ansley phys TB/linear_speed_phys_tb.sv"},
        {"module": "over_I_tb", "path": "../Ansley phys TB/over_I_phys_tb.sv"},
        {"module": "Segway_tb", "path": "../Ansley phys TB/Segway_phys_tb.sv"},
        {"module": "steering_response_tb", "path": "../Ansley phys TB/steering_response_phys_tb.sv"},
    ],
}


# Make output look pretty
class Color:
    """ANSI color codes for terminal output"""
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class TestRunner:
    def __init__(self, project_dir):
        self.project_dir = Path(project_dir)
        self.results = []
        self.start_time = None
        self.end_time = None
        
    def print_header(self, message):
        """Print a formatted header"""
        print(f"\n{Color.HEADER}{Color.BOLD}{'='*70}{Color.ENDC}")
        print(f"{Color.HEADER}{Color.BOLD}{message.center(70)}{Color.ENDC}")
        print(f"{Color.HEADER}{Color.BOLD}{'='*70}{Color.ENDC}\n")
    
    def print_step(self, message):
        """Print a step message"""
        print(f"{Color.OKCYAN}▶ {message}{Color.ENDC}")
    
    def print_success(self, message):
        """Print a success message"""
        print(f"{Color.OKGREEN}✓ {message}{Color.ENDC}")
    
    def print_error(self, message):
        """Print an error message"""
        print(f"{Color.FAIL}✗ {message}{Color.ENDC}")
    
    def print_warning(self, message):
        """Print a warning message"""
        print(f"{Color.WARNING}⚠ {message}{Color.ENDC}")
    
    def run_command(self, cmd, description="", capture_output=True):
        """Run a shell command and return success status"""
        if description:
            self.print_step(description)
        
        try:
            if capture_output:
                result = subprocess.run(
                    cmd,
                    shell=True,
                    capture_output=True,
                    text=True,
                    cwd=self.project_dir
                )
            else:
                result = subprocess.run(
                    cmd,
                    shell=True,
                    cwd=self.project_dir
                )
            
            if result.returncode == 0:
                return True, result.stdout if capture_output else ""
            else:
                if capture_output and result.stderr:
                    print(result.stderr)
                return False, result.stderr if capture_output else ""
        except Exception as e:
            self.print_error(f"Command failed: {e}")
            return False, str(e)
    
    def check_modelsim(self):
        """Check if ModelSim/QuestaSim is available"""
        self.print_step("Checking for ModelSim/QuestaSim installation...")
        
        success, output = self.run_command("vlog -version", capture_output=True)
        if success:
            self.print_success(f"Found ModelSim/QuestaSim")
            return True
        else:
            self.print_error("ModelSim/QuestaSim not found in PATH")
            self.print_warning("Please ensure ModelSim or QuestaSim is installed and in your PATH")
            return False
    
    def create_work_library(self):
        """Create/recreate the work library"""
        self.print_step("Creating work library...")
        
        # Remove existing work library if it exists
        work_path = self.project_dir / WORK_LIB
        if work_path.exists():
            self.print_step(f"Removing existing {WORK_LIB} library...")
            success, _ = self.run_command(f"vdel -all -lib {WORK_LIB}")
        
        # Create new work library
        success, _ = self.run_command(f"vlib {WORK_LIB}")
        if success:
            self.print_success(f"Created {WORK_LIB} library")
            return True
        else:
            self.print_error(f"Failed to create {WORK_LIB} library")
            return False
    
    def compile_design(self):
        """Compile all design files"""
        self.print_header("Compiling Design Files")
        
        all_files = COMPILE_ORDER.copy()
        
        failed_files = []
        compiled_count = 0
        
        for file in all_files:
            file_path = self.project_dir / file
            if not file_path.exists():
                self.print_warning(f"File not found: {file}, skipping...")
                continue
            
            self.print_step(f"Compiling {file}...")
            success, output = self.run_command(
                f"vlog -sv -work {WORK_LIB} {file}",
                capture_output=True
            )
            
            if success:
                self.print_success(f"✓ {file}")
                compiled_count += 1
            else:
                self.print_error(f"✗ {file}")
                failed_files.append(file)
                if output:
                    print(f"  Error output:\n{output}")
        
        if failed_files:
            self.print_error(f"\nFailed to compile {len(failed_files)} file(s)")
            return False
        
        self.print_success(f"\nSuccessfully compiled {compiled_count} design file(s)")
        return True
    
    def run_testbench(self, tb_module, tb_path, display_name=None):
        """Compile and run a single testbench
        
        Args:
            tb_module: The actual module name inside the .sv file
            tb_path: Path to the testbench file
            display_name: Optional name for display (defaults to tb_path basename)
        """
        if display_name is None:
            display_name = Path(tb_path).stem
        
        print(f"\n{Color.OKBLUE}{'─'*70}{Color.ENDC}")
        print(f"{Color.OKBLUE}{Color.BOLD}Running: {display_name}{Color.ENDC}")
        print(f"{Color.OKBLUE}{'─'*70}{Color.ENDC}")
        
        start = datetime.now()
        
        # Compile testbench
        self.print_step(f"Compiling {tb_path}...")
        success, output = self.run_command(
            f"vlog -sv -work {WORK_LIB} {tb_path}",
            capture_output=True
        )
        
        if not success:
            self.print_error(f"Failed to compile {display_name}")
            if output:
                print(output)
            end = datetime.now()
            return {
                "name": display_name,
                "status": "COMPILE_FAILED",
                "duration": (end - start).total_seconds(),
                "error": output
            }
        
        self.print_success(f"Compiled {display_name}")
        
        # Run simulation using the actual module name
        self.print_step(f"Running simulation (module: {tb_module})...")
        # Use onfinish stop to prevent auto-quit, then explicitly quit
        # This gives us better control and captures all output
        success, output = self.run_command(
            f"vsim {VSIM_FLAGS} -do \"run -all; quit -f\" {WORK_LIB}.{tb_module}",
            capture_output=True
        )
        
        end = datetime.now()
        duration = (end - start).total_seconds()
        
        # Check for common success/failure patterns in output
        if output:
            output_lower = output.lower()
            if "error" in output_lower and "0 errors" not in output_lower:
                status = "FAILED"
            elif "yahoo" in output_lower or "test passed" in output_lower or "success" in output_lower:
                status = "PASSED"
            elif success:
                status = "COMPLETED"
            else:
                status = "FAILED"
        else:
            status = "COMPLETED" if success else "FAILED"
        
        if status == "PASSED":
            self.print_success(f"✓ {display_name} PASSED ({duration:.2f}s)")
        elif status == "FAILED":
            self.print_error(f"✗ {display_name} FAILED ({duration:.2f}s)")
        else:
            self.print_warning(f"○ {display_name} COMPLETED ({duration:.2f}s)")
        
        return {
            "name": display_name,
            "module": tb_module,
            "status": status,
            "duration": duration,
            "output": output
        }
    
    def run_all_testbenches(self, test_set="tests"):
        """Run all testbenches from the specified set"""
        if test_set not in TESTBENCH_SETS:
            self.print_error(f"Unknown test set: {test_set}")
            self.print_warning(f"Available sets: {', '.join(TESTBENCH_SETS.keys())}")
            return
        
        testbenches = TESTBENCH_SETS[test_set]
        self.print_header(f"Running Testbenches from '{test_set}' ({len(testbenches)} tests)")
        
        for tb in testbenches:
            # Use the file basename as display name
            display_name = Path(tb["path"]).stem
            result = self.run_testbench(tb["module"], tb["path"], display_name)
            self.results.append(result)
    
    def print_summary(self):
        """Print test summary"""
        self.print_header("Test Summary")
        
        passed = sum(1 for r in self.results if r["status"] == "PASSED")
        failed = sum(1 for r in self.results if r["status"] == "FAILED")
        compile_failed = sum(1 for r in self.results if r["status"] == "COMPILE_FAILED")
        completed = sum(1 for r in self.results if r["status"] == "COMPLETED")
        total = len(self.results)
        
        total_duration = sum(r["duration"] for r in self.results)
        
        print(f"{'Testbench':<30} {'Status':<15} {'Duration':>10}")
        print(f"{'-'*57}")
        
        for result in self.results:
            name = result["name"]
            status = result["status"]
            duration = result["duration"]
            
            if status == "PASSED":
                color = Color.OKGREEN
                symbol = "✓"
            elif status == "FAILED":
                color = Color.FAIL
                symbol = "✗"
            elif status == "COMPILE_FAILED":
                color = Color.FAIL
                symbol = "✗"
            else:
                color = Color.WARNING
                symbol = "○"
            
            print(f"{name:<30} {color}{symbol} {status:<14}{Color.ENDC} {duration:>8.2f}s")
        
        print(f"{'-'*57}")
        print(f"\n{Color.BOLD}Results:{Color.ENDC}")
        print(f"  {Color.OKGREEN}Passed:{Color.ENDC}          {passed}/{total}")
        print(f"  {Color.FAIL}Failed:{Color.ENDC}          {failed}/{total}")
        print(f"  {Color.FAIL}Compile Failed:{Color.ENDC}  {compile_failed}/{total}")
        print(f"  {Color.WARNING}Completed:{Color.ENDC}       {completed}/{total}")
        print(f"\n{Color.BOLD}Total Duration:{Color.ENDC}   {total_duration:.2f}s")
        
        # Overall status
        if compile_failed > 0 or failed > 0:
            print(f"\n{Color.FAIL}{Color.BOLD}OVERALL: FAILURES DETECTED{Color.ENDC}")
            return False
        elif passed > 0:
            print(f"\n{Color.OKGREEN}{Color.BOLD}OVERALL: ALL TESTS PASSED{Color.ENDC}")
            return True
        else:
            print(f"\n{Color.WARNING}{Color.BOLD}OVERALL: TESTS COMPLETED{Color.ENDC}")
            return True
    
    def save_results(self, filename="test_results.json"):
        """Save test results to JSON file"""
        results_file = self.project_dir / filename
        
        report = {
            "timestamp": self.start_time.isoformat() if self.start_time else None,
            "duration": (self.end_time - self.start_time).total_seconds() if self.end_time and self.start_time else 0,
            "results": self.results
        }
        
        with open(results_file, 'w') as f:
            json.dump(report, f, indent=2)
        
        self.print_step(f"Results saved to {filename}")
    
    def run(self, skip_compile=False, test_set="tests"):
        """Main execution flow"""
        self.start_time = datetime.now()
        
        self.print_header("Segway Testbench Runner")
        print(f"Project Directory: {self.project_dir}")
        print(f"Test Set: {test_set}")
        print(f"Start Time: {self.start_time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        
        # Check for ModelSim
        if not self.check_modelsim():
            return False
        
        if not skip_compile:
            # Create work library
            if not self.create_work_library():
                return False
            
            # Compile design
            if not self.compile_design():
                self.print_error("Design compilation failed. Cannot continue.")
                return False
        else:
            self.print_warning("Skipping compilation (using existing work library)")
        
        # Run all testbenches
        self.run_all_testbenches(test_set=test_set)
        
        self.end_time = datetime.now()
        
        # Print summary
        success = self.print_summary()
        
        # Save results
        self.save_results()
        
        return success


def interactive_menu():
    """Display interactive menu for test set selection"""
    print(f"\n{Color.HEADER}{Color.BOLD}{'='*70}{Color.ENDC}")
    print(f"{Color.HEADER}{Color.BOLD}{'Segway Testbench Runner - Interactive Mode'.center(70)}{Color.ENDC}")
    print(f"{Color.HEADER}{Color.BOLD}{'='*70}{Color.ENDC}\n")
    
    print("Available test sets:\n")
    test_set_list = list(TESTBENCH_SETS.keys())
    
    for i, test_set in enumerate(test_set_list, 1):
        count = len(TESTBENCH_SETS[test_set])
        print(f"  {Color.OKCYAN}{i}.{Color.ENDC} {test_set:<20} ({count} testbenches)")
    
    print(f"  {Color.OKCYAN}{len(test_set_list) + 1}.{Color.ENDC} {'all':<20} (run all test sets)")
    print(f"  {Color.FAIL}0.{Color.ENDC} Exit\n")
    
    while True:
        try:
            choice = input(f"{Color.BOLD}Select test set [1-{len(test_set_list) + 1}, 0 to exit]: {Color.ENDC}")
            choice_num = int(choice)
            
            if choice_num == 0:
                print("Exiting...")
                return None
            elif 1 <= choice_num <= len(test_set_list):
                return test_set_list[choice_num - 1]
            elif choice_num == len(test_set_list) + 1:
                return "all"
            else:
                print(f"{Color.FAIL}Invalid choice. Please try again.{Color.ENDC}\n")
        except (ValueError, EOFError, KeyboardInterrupt):
            print(f"\n{Color.FAIL}Invalid input or interrupted. Exiting...{Color.ENDC}")
            return None


def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Run all Segway testbenches",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=f"""
Available test sets:
  tests          - Basic testbenches in tests/ directory
  physics_tests  - Physics-based testbenches in physics_tests/ directory
  ansley         - Ansley's physics testbenches in 'Ansley phys TB/' directory
  all            - Run all test sets sequentially

Examples:
  python run_all_testbenches.py                    # Interactive mode
  python run_all_testbenches.py --set tests        # Run specific set
  python run_all_testbenches.py --set all          # Run all sets
  python run_all_testbenches.py --skip-compile --set ansley
"""
    )
    parser.add_argument(
        "--skip-compile",
        action="store_true",
        help="Skip compilation and use existing work library"
    )
    parser.add_argument(
        "--dir",
        default=".",
        help="Project directory (default: current directory)"
    )
    parser.add_argument(
        "--set",
        choices=list(TESTBENCH_SETS.keys()) + ["all"],
        default=None,
        help="Which test set to run (omit for interactive menu)"
    )
    
    args = parser.parse_args()
    
    project_dir = Path(args.dir).resolve()
    
    if not project_dir.exists():
        print(f"Error: Directory '{project_dir}' does not exist")
        sys.exit(1)
    
    # Interactive mode if no --set specified
    if args.set is None:
        selected_set = interactive_menu()
        if selected_set is None:
            sys.exit(0)
    else:
        selected_set = args.set
    
    # Handle 'all' option
    if selected_set == "all":
        overall_success = True
        for test_set in TESTBENCH_SETS.keys():
            print(f"\n{'='*70}")
            print(f"Running test set: {test_set}")
            print(f"{'='*70}\n")
            
            runner = TestRunner(project_dir)
            # Only compile on first run
            skip_compile = args.skip_compile or (test_set != list(TESTBENCH_SETS.keys())[0])
            success = runner.run(skip_compile=skip_compile, test_set=test_set)
            
            if not success:
                overall_success = False
        
        sys.exit(0 if overall_success else 1)
    else:
        runner = TestRunner(project_dir)
        success = runner.run(skip_compile=args.skip_compile, test_set=selected_set)
        sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()

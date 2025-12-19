
#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import subprocess
import os
import sys
from pathlib import Path
import yaml

class SocRATESGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("SocRATES - Social Navigation Scenario Generator")
        self.root.geometry("1000x850")

        # Set color scheme
        self.bg_color = "#1e1e2e"
        self.accent_color = "#89b4fa"
        self.secondary_color = "#313244"
        self.text_color = "#cdd6f4"
        self.success_color = "#a6e3a1"
        self.warning_color = "#f9e2af"
        self.error_color = "#f38ba8"

        # Configure root background
        self.root.configure(bg=self.bg_color)

        # Get the SoNaS directory (where this script is located)
        self.sonas_dir = Path(__file__).parent.absolute()

        # Configure modern style
        self.setup_styles()

        # Create main frame with custom background
        main_frame = tk.Frame(root, bg=self.bg_color, padx=20, pady=20)
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Configure grid weights
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(1, weight=0)
        main_frame.rowconfigure(3, weight=1)

        # Header frame with gradient effect
        header_frame = tk.Frame(main_frame, bg=self.secondary_color, relief=tk.FLAT, bd=0)
        header_frame.grid(row=0, column=0, pady=(0, 20), sticky=(tk.W, tk.E))
        header_frame.columnconfigure(0, weight=1)

        # Title with subtitle
        title_label = tk.Label(header_frame, text="SocRATES",
                              font=('Helvetica', 28, 'bold'),
                              bg=self.secondary_color, fg=self.accent_color)
        title_label.grid(row=0, column=0, pady=(15, 5))

        # subtitle_label = tk.Label(header_frame,
        #                          text="Social Navigation Scenario Generator",
        #                          font=('Helvetica', 12),
        #                          bg=self.secondary_color, fg=self.text_color)
        # subtitle_label.grid(row=1, column=0, pady=(0, 15))

        # Button frame with better spacing
        button_frame = tk.Frame(main_frame, bg=self.bg_color)
        button_frame.grid(row=1, column=0, pady=20, sticky=(tk.W, tk.E))
        button_frame.columnconfigure(0, weight=1)
        button_frame.columnconfigure(1, weight=1)

        # Create styled buttons in a 2x2 grid
        self.btn_annotate = self.create_styled_button(
            button_frame, "Annotate Map", self.annotate_map,
            self.accent_color, row=0, col=0)

        self.btn_configure = self.create_styled_button(
            button_frame, "Configure Scenario", self.configure_scenario,
            "#cba6f7", row=0, col=1)

        self.btn_generate = self.create_styled_button(
            button_frame, "Generate Scenario", self.generate_scenario,
            self.success_color, row=1, col=0)

        self.btn_simulate = self.create_styled_button(
            button_frame, "Simulate", self.simulate,
            "#fab387", row=1, col=1)

        # Utility buttons
        self.btn_view_scenegraph = self.create_styled_button(
            button_frame, "View Scene Graph", self.view_scene_graph,
            self.warning_color, row=2, col=0)

        self.btn_killros = self.create_styled_button(
            button_frame, "Kill ROS", self.kill_ros,
            self.error_color, row=2, col=1)

        self.btn_rqt_publisher = self.create_styled_button(
            button_frame, "RQT Topic Publisher", self.launch_rqt_publisher,
            "#94e2d5", row=3, col=0, colspan=2)

        # Status indicator with icon
        status_frame = tk.Frame(main_frame, bg=self.secondary_color, relief=tk.FLAT)
        status_frame.grid(row=2, column=0, pady=10, sticky=(tk.W, tk.E))

        status_icon = tk.Label(status_frame, text="●", font=('Arial', 12),
                              bg=self.secondary_color, fg=self.success_color)
        status_icon.pack(side=tk.LEFT, padx=(10, 5))

        self.status_var = tk.StringVar(value="Ready")
        status_label = tk.Label(status_frame, textvariable=self.status_var,
                               font=('Helvetica', 11),
                               bg=self.secondary_color, fg=self.text_color)
        status_label.pack(side=tk.LEFT, padx=5)

        # Output console with modern styling
        console_container = tk.Frame(main_frame, bg=self.bg_color)
        console_container.grid(row=3, column=0, pady=10, sticky=(tk.W, tk.E, tk.N, tk.S))
        console_container.columnconfigure(0, weight=1)
        console_container.rowconfigure(1, weight=1)

        console_header = tk.Label(console_container, text="Output Console",
                                 font=('Helvetica', 11, 'bold'),
                                 bg=self.bg_color, fg=self.text_color,
                                 anchor='w')
        console_header.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(0, 5))

        console_frame = tk.Frame(console_container, bg=self.secondary_color,
                                relief=tk.FLAT, bd=2)
        console_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        console_frame.columnconfigure(0, weight=1)
        console_frame.rowconfigure(0, weight=1)

        self.console = scrolledtext.ScrolledText(
            console_frame, height=12,
            font=('Consolas', 10) if sys.platform == 'win32' else ('Courier', 10),
            bg=self.secondary_color, fg=self.text_color,
            insertbackground=self.accent_color,
            selectbackground=self.accent_color,
            selectforeground=self.bg_color,
            relief=tk.FLAT, bd=0, padx=10, pady=10)
        self.console.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Control buttons
        control_frame = tk.Frame(main_frame, bg=self.bg_color)
        control_frame.grid(row=4, column=0, pady=10)

        self.btn_clear = tk.Button(control_frame, text="Clear Console",
                                   command=self.clear_console,
                                   bg=self.secondary_color, fg=self.text_color,
                                   font=('Helvetica', 10),
                                   relief=tk.FLAT, bd=0,
                                   padx=15, pady=8,
                                   cursor='hand2',
                                   activebackground=self.accent_color,
                                   activeforeground=self.bg_color)
        self.btn_clear.pack(side=tk.LEFT, padx=5)

        # Add hover effect
        self.btn_clear.bind('<Enter>', lambda e: self.btn_clear.config(bg=self.accent_color, fg=self.bg_color))
        self.btn_clear.bind('<Leave>', lambda e: self.btn_clear.config(bg=self.secondary_color, fg=self.text_color))

        self.log("SocRATES GUI initialized")
        # self.log(f"Working directory: {self.sonas_dir}")

    def setup_styles(self):
        """Configure ttk styles"""
        style = ttk.Style()
        style.theme_use('default')

    def create_styled_button(self, parent, text, command, color, row, col, colspan=1):
        """Create a modern styled button"""
        btn = tk.Button(parent, text=text,
                       command=command,
                       bg=color, fg=self.bg_color,
                       font=('Helvetica', 12, 'bold'),
                       relief=tk.FLAT, bd=0,
                       padx=20, pady=20,
                       cursor='hand2',
                       activebackground=self.lighten_color(color),
                       activeforeground=self.bg_color)
        btn.grid(row=row, column=col, columnspan=colspan,
                padx=8, pady=8, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Add hover effects
        original_color = color
        lighter_color = self.lighten_color(color)
        btn.bind('<Enter>', lambda e: btn.config(bg=lighter_color))
        btn.bind('<Leave>', lambda e: btn.config(bg=original_color))

        return btn

    def lighten_color(self, hex_color):
        """Lighten a hex color by 10%"""
        hex_color = hex_color.lstrip('#')
        r, g, b = tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))
        r = min(255, int(r * 1.1))
        g = min(255, int(g * 1.1))
        b = min(255, int(b * 1.1))
        return f'#{r:02x}{g:02x}{b:02x}'

    def log(self, message):
        """Add message to console"""
        self.console.insert(tk.END, f"{message}\n")
        self.console.see(tk.END)
        self.root.update_idletasks()

    def clear_console(self):
        """Clear the console output"""
        self.console.delete(1.0, tk.END)

    def annotate_map(self):
        """Launch the map annotator"""
        self.log("\n" + "="*50)
        self.log("Preparing Map Annotator...")

        # Ask if user wants to edit config.yaml first
        response = messagebox.askyesnocancel(
            "Configure Annotator",
            "Would you like to edit the annotator configuration (config.yaml) first?\n\n"
            "The config file contains:\n"
            "- Map image paths\n"
            "- Scene graph schema (node and edge types)\n"
            "- Zoom and font size settings\n\n"
            "Yes - Edit config.yaml first\n"
            "No - Launch annotator directly\n"
            "Cancel - Go back"
        )

        if response is None:  # User clicked Cancel
            self.log("Annotation cancelled by user")
            self.status_var.set("Ready")
            return

        if response:  # User clicked Yes - wants to edit config
            config_path = self.sonas_dir / "annotator" / "config.yaml"

            if not config_path.exists():
                messagebox.showerror("Error", f"config.yaml not found at {config_path}")
                self.log(f"ERROR: config.yaml not found")
                return

            # Open config.yaml in default editor
            try:
                self.log(f"Opening {config_path} for editing...")
                if sys.platform == 'linux':
                    subprocess.Popen(['xdg-open', str(config_path)])
                elif sys.platform == 'darwin':
                    subprocess.Popen(['open', str(config_path)])
                elif sys.platform == 'win32':
                    os.startfile(str(config_path))

                self.log("Please save and close the config file when done")

                # Ask if ready to proceed after editing
                proceed = messagebox.askyesno(
                    "Continue?",
                    "Have you finished editing the config file?\n\n"
                    "Click Yes to launch the annotator\n"
                    "Click No to cancel"
                )

                if not proceed:
                    self.log("Annotation cancelled after config editing")
                    self.status_var.set("Ready")
                    return

            except Exception as e:
                messagebox.showerror("Error", f"Could not open config.yaml:\n{str(e)}")
                self.log(f"ERROR: Could not open config.yaml - {str(e)}")
                return

        # Proceed with launching annotator
        self.log("Launching Map Annotator...")
        self.status_var.set("Running annotator...")

        annotator_path = self.sonas_dir / "annotator" / "annotator.py"

        if not annotator_path.exists():
            messagebox.showerror("Error", f"Annotator not found at {annotator_path}")
            self.log(f"ERROR: Annotator not found at {annotator_path}")
            self.status_var.set("Error: Annotator not found")
            return

        try:
            # Build the command to run in the new terminal
            annotator_dir = self.sonas_dir / "annotator"
            command = f"cd '{annotator_dir}' && python3 annotator.py"

            # Open a new terminal window based on the platform
            if sys.platform == 'linux':
                # Try common Linux terminal emulators
                terminals = [
                    ['gnome-terminal', '--', 'bash', '-c', f'{command}; echo "Press Enter to close..."; read'],
                    ['xterm', '-e', f'bash -c "{command}; echo \\"Press Enter to close...\\"; read"'],
                    ['konsole', '-e', 'bash', '-c', f'{command}; echo "Press Enter to close..."; read'],
                    ['xfce4-terminal', '-e', f'bash -c "{command}; echo \\"Press Enter to close...\\"; read"'],
                ]

                terminal_opened = False
                for term_cmd in terminals:
                    try:
                        subprocess.Popen(term_cmd)
                        terminal_opened = True
                        self.log(f"Opened new terminal for map annotator")
                        self.log(f"Running: python3 annotator.py")
                        self.status_var.set("Annotator running in terminal")
                        break
                    except FileNotFoundError:
                        continue

                if not terminal_opened:
                    raise Exception("No suitable terminal emulator found")

            elif sys.platform == 'darwin':
                # macOS
                script = f'tell application "Terminal" to do script "cd \\"{annotator_dir}\\" && python3 annotator.py"'
                subprocess.Popen(['osascript', '-e', script])
                self.log("Opened new terminal for map annotator")
                self.status_var.set("Annotator running in terminal")

            elif sys.platform == 'win32':
                # Windows
                subprocess.Popen(['start', 'cmd', '/k', f'cd /d "{annotator_dir}" && python annotator.py'], shell=True)
                self.log("Opened new terminal for map annotator")
                self.status_var.set("Annotator running in terminal")

        except Exception as e:
            self.log(f"ERROR: Could not open terminal - {str(e)}")
            self.status_var.set("Error occurred")
            messagebox.showerror("Error", f"Could not open terminal:\n{str(e)}\n\nPlease run manually:\ncd annotator && python3 annotator.py")

    def configure_scenario(self):
        """Open inputs.yaml for editing"""
        self.log("\n" + "="*50)
        self.log("Opening configuration file...")

        inputs_path = self.sonas_dir / "inputs.yaml"

        if not inputs_path.exists():
            messagebox.showerror("Error", f"inputs.yaml not found at {inputs_path}")
            self.log(f"ERROR: inputs.yaml not found")
            return

        # Try to open with default editor
        try:
            if sys.platform == 'linux':
                subprocess.Popen(['xdg-open', str(inputs_path)])
            elif sys.platform == 'darwin':
                subprocess.Popen(['open', str(inputs_path)])
            elif sys.platform == 'win32':
                os.startfile(str(inputs_path))

            # self.log(f"Opened {inputs_path} in default editor")
            self.status_var.set("Configuration file opened")

        except Exception as e:
            # Fallback: show path and ask user to open manually
            messagebox.showinfo("Configuration File",
                              f"Please edit the file:\n{inputs_path}")
            self.log(f"Configuration file location: {inputs_path}")
            self.status_var.set("Configuration file path displayed")

    def view_scene_graph(self):
        """Open the scene graph PNG file specified in annotator config"""
        self.log("\n" + "="*50)
        self.log("Opening scene graph visualization...")

        config_path = self.sonas_dir / "annotator" / "config.yaml"

        if not config_path.exists():
            messagebox.showerror("Error", f"Annotator config not found at {config_path}")
            self.log(f"ERROR: config.yaml not found")
            return

        try:
            # Read the config file to get scene graph PNG path
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            scene_graph_rel_path = config.get('out')
            if not scene_graph_rel_path:
                messagebox.showerror("Error", "Scene graph output path (out) not found in config.yaml")
                self.log("ERROR: 'out' field not found in config.yaml")
                return

            # Construct full path (relative to annotator directory)
            scene_graph_path = self.sonas_dir / "annotator" / scene_graph_rel_path

            if not scene_graph_path.exists():
                messagebox.showerror("Error", f"Scene graph PNG not found:\n{scene_graph_path}\n\nPlease run the annotator first to generate it.")
                self.log(f"ERROR: Scene graph PNG not found at {scene_graph_path}")
                return

            # Open with default image viewer
            if sys.platform == 'linux':
                subprocess.Popen(['xdg-open', str(scene_graph_path)])
            elif sys.platform == 'darwin':
                subprocess.Popen(['open', str(scene_graph_path)])
            elif sys.platform == 'win32':
                os.startfile(str(scene_graph_path))

            self.log(f"Opened scene graph PNG: {os.path.relpath(scene_graph_path)}")
            self.status_var.set("Scene graph visualization opened")

        except yaml.YAMLError as e:
            messagebox.showerror("Error", f"Could not parse config.yaml:\n{str(e)}")
            self.log(f"ERROR: Could not parse config.yaml - {str(e)}")
        except Exception as e:
            messagebox.showerror("Error", f"Could not open scene graph:\n{str(e)}")
            self.log(f"ERROR: {str(e)}")

    def generate_scenario(self):
        """Run scenario generation in a new terminal"""
        self.log("\n" + "="*50)
        self.log("Starting scenario generation...")
        self.status_var.set("Generating scenario...")

        run_socrates = self.sonas_dir / "run_socrates.py"

        if not run_socrates.exists():
            messagebox.showerror("Error", f"run_socrates.py not found at {run_socrates}")
            self.log(f"ERROR: run_socrates.py not found")
            self.status_var.set("Error: run_socrates.py not found")
            return

        try:
            # Build the command to run in the new terminal
            command = f"cd '{self.sonas_dir}' && python3 run_socrates.py --mode gen"

            # Open a new terminal window based on the platform
            if sys.platform == 'linux':
                # Try common Linux terminal emulators
                terminals = [
                    ['gnome-terminal', '--', 'bash', '-c', f'{command}; echo "Press Enter to close..."; read'],
                    ['xterm', '-e', f'bash -c "{command}; echo \\"Press Enter to close...\\"; read"'],
                    ['konsole', '-e', 'bash', '-c', f'{command}; echo "Press Enter to close..."; read'],
                    ['xfce4-terminal', '-e', f'bash -c "{command}; echo \\"Press Enter to close...\\"; read"'],
                ]

                terminal_opened = False
                for term_cmd in terminals:
                    try:
                        subprocess.Popen(term_cmd)
                        terminal_opened = True
                        self.log(f"Opened new terminal for scenario generation")
                        self.log(f"Running: python3 run_socrates.py --mode gen")
                        self.status_var.set("Scenario generation running in terminal")
                        break
                    except FileNotFoundError:
                        continue

                if not terminal_opened:
                    raise Exception("No suitable terminal emulator found")

            elif sys.platform == 'darwin':
                # macOS
                script = f'tell application "Terminal" to do script "cd \\"{self.sonas_dir}\\" && python3 run_socrates.py --mode gen"'
                subprocess.Popen(['osascript', '-e', script])
                self.log("Opened new terminal for scenario generation")
                self.status_var.set("Scenario generation running in terminal")

            elif sys.platform == 'win32':
                # Windows
                subprocess.Popen(['start', 'cmd', '/k', f'cd /d "{self.sonas_dir}" && python run_socrates.py --mode gen'], shell=True)
                self.log("Opened new terminal for scenario generation")
                self.status_var.set("Scenario generation running in terminal")

        except Exception as e:
            self.log(f"ERROR: Could not open terminal - {str(e)}")
            self.status_var.set("Error occurred")
            messagebox.showerror("Error", f"Could not open terminal:\n{str(e)}\n\nPlease run manually:\npython3 run_socrates.py --mode gen")

    def simulate(self):
        """Launch simulation in a new terminal"""
        self.log("\n" + "="*50)
        self.log("Launching simulation...")
        self.status_var.set("Running simulation...")

        run_socrates = self.sonas_dir / "run_socrates.py"

        if not run_socrates.exists():
            messagebox.showerror("Error", f"run_socrates.py not found at {run_socrates}")
            self.log(f"ERROR: run_socrates.py not found")
            self.status_var.set("Error: run_socrates.py not found")
            return

        try:
            # Build the command to run in the new terminal
            command = f"cd '{self.sonas_dir}' && python3 run_socrates.py --mode sim"

            # Open a new terminal window based on the platform
            if sys.platform == 'linux':
                # Try common Linux terminal emulators
                terminals = [
                    ['gnome-terminal', '--', 'bash', '-c', f'{command}; echo "Press Enter to close..."; read'],
                    ['xterm', '-e', f'bash -c "{command}; echo \\"Press Enter to close...\\"; read"'],
                    ['konsole', '-e', 'bash', '-c', f'{command}; echo "Press Enter to close..."; read'],
                    ['xfce4-terminal', '-e', f'bash -c "{command}; echo \\"Press Enter to close...\\"; read"'],
                ]

                terminal_opened = False
                for term_cmd in terminals:
                    try:
                        subprocess.Popen(term_cmd)
                        terminal_opened = True
                        self.log(f"Opened new terminal for simulation")
                        self.log(f"Running: python3 run_socrates.py --mode sim")
                        self.status_var.set("Simulation running in terminal")
                        break
                    except FileNotFoundError:
                        continue

                if not terminal_opened:
                    raise Exception("No suitable terminal emulator found")

            elif sys.platform == 'darwin':
                # macOS
                script = f'tell application "Terminal" to do script "cd \\"{self.sonas_dir}\\" && python3 run_socrates.py --mode sim"'
                subprocess.Popen(['osascript', '-e', script])
                self.log("Opened new terminal for simulation")
                self.status_var.set("Simulation running in terminal")

            elif sys.platform == 'win32':
                # Windows
                subprocess.Popen(['start', 'cmd', '/k', f'cd /d "{self.sonas_dir}" && python run_socrates.py --mode sim'], shell=True)
                self.log("Opened new terminal for simulation")
                self.status_var.set("Simulation running in terminal")

        except Exception as e:
            self.log(f"ERROR: Could not open terminal - {str(e)}")
            self.status_var.set("Error occurred")
            messagebox.showerror("Error", f"Could not open terminal:\n{str(e)}\n\nPlease run manually:\npython3 run_socrates.py --mode sim")
    def kill_ros(self):
        """Kill all ROS processes"""
        self.log("\n" + "="*50)
        self.log("Killing ROS processes...")
        self.status_var.set("Killing ROS...")

        try:
            # Use the killros2 approach: grep for all ros/gazebo processes and kill them
            # This is more comprehensive and catches all processes with ros/gazebo in the name
            command = """
# Kill all ROS processes
ps aux | grep ros | grep -v grep | awk '{print $2}' | xargs -r kill -9 2>/dev/null
# Kill all Gazebo processes
ps aux | grep gazebo | grep -v grep | awk '{print $2}' | xargs -r kill -9 2>/dev/null
# Also try specific process names
killall -9 gzserver gzclient 2>/dev/null
"""

            # Run command through bash
            result = subprocess.run(['bash', '-c', command],
                                  capture_output=True,
                                  text=True,
                                  timeout=10)

            self.log("ROS kill commands executed (SIGKILL)")

            # Check if any processes were killed by looking for remaining ros/gazebo processes
            check_cmd = "ps aux | grep -E 'ros|gazebo' | grep -v grep | wc -l"
            check_result = subprocess.run(['bash', '-c', check_cmd],
                                        capture_output=True,
                                        text=True,
                                        timeout=5)

            remaining = int(check_result.stdout.strip())
            if remaining > 0:
                self.log(f"Warning: {remaining} ROS/Gazebo processes may still be running")
                # List remaining processes
                list_cmd = "ps aux | grep -E 'ros|gazebo' | grep -v grep"
                list_result = subprocess.run(['bash', '-c', list_cmd],
                                           capture_output=True,
                                           text=True,
                                           timeout=5)
                self.log("Remaining processes:")
                self.log(list_result.stdout)
            else:
                self.log("All ROS/Gazebo processes have been terminated")

            self.status_var.set("ROS killed")
            #messagebox.showinfo("Success", "Kill command executed")

        except subprocess.TimeoutExpired:
            self.log("ERROR: killros command timed out")
            self.status_var.set("Error: timeout")
            messagebox.showerror("Error", "killros command timed out")
        except Exception as e:
            self.log(f"ERROR: {str(e)}")
            self.status_var.set("Error occurred")
            messagebox.showerror("Error", f"Could not kill ROS:\n{str(e)}")

    def launch_rqt_publisher(self):
        """Launch RQT topic publisher"""
        self.log("\n" + "="*50)
        self.log("Launching RQT Topic Publisher...")
        self.status_var.set("Launching RQT...")

        try:
            # Launch rqt_publisher directly (it's a GUI application)
            subprocess.Popen(['ros2', 'run', 'rqt_publisher', 'rqt_publisher'],
                           stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL)

            self.log("RQT Topic Publisher launched")
            self.status_var.set("RQT publisher running")

        except FileNotFoundError:
            self.log("ERROR: ros2 or rqt_publisher not found")
            self.status_var.set("Error: rqt_publisher not found")
            messagebox.showerror("Error", "Could not launch rqt_publisher.\n\nMake sure ROS2 and rqt_publisher are installed and in your PATH.")
        except Exception as e:
            self.log(f"ERROR: {str(e)}")
            self.status_var.set("Error occurred")
            messagebox.showerror("Error", f"Could not launch rqt_publisher:\n{str(e)}")

def main():
    root = tk.Tk()
    app = SocRATESGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()

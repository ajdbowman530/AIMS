import sys
import os
import csv
from pathlib import Path
import numpy as np

from PySide6.QtWidgets import QApplication, QVBoxLayout, QFileDialog, QWidget, QInputDialog
from PySide6.QtCore import Qt
from PySide6.QtUiTools import QUiLoader

import matplotlib
matplotlib.use('QtAgg') 
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

class TelemetryVisualizer(object):
    def __init__(self, initial_file_path=None):
        # Telemetry Data Cache
        self.data_dict = {}
        self.time_vec = None
        self.headers = []
        
        # Dynamically load the layout file
        loader = QUiLoader()
        ui_path = Path(__file__).resolve().parent / "main_window.ui"
        self.ui = loader.load(str(ui_path), None)
        self.ui.setWindowTitle("AIMS - Flight Telemetry Post-Processor")

        # Setup the Matplotlib Canvas
        canvas_layout = QVBoxLayout(self.ui.canvas_widget)
        self.fig = Figure(figsize=(10, 8), dpi=100)
        self.canvas = FigureCanvas(self.fig)
        canvas_layout.addWidget(self.canvas)
        
        # Pre-generate 4 sharex subplots stacked vertically
        self.axs = self.fig.subplots(4, 1, sharex=True)
        for ax in self.axs:
            ax.grid(True, linestyle=':', alpha=0.6)
            ax.set_ylabel("No Data Selected")
        self.axs[3].set_xlabel("Time (seconds)")

        self.plot_configs = [
            {
                "box": self.ui.lon_box, 
                "list": self.ui.list_p1, 
                "combo": self.ui.combo_add_state_1, 
                "btn": self.ui.add_p1, 
                "axis_idx": 0, 
                "defaults": ['q_actual', 'q_cmd'],
                "settings": {
                    "grid": True,
                    "custom_label": "",
                    "xlim_auto": True,
                    "xmin": 0.0,
                    "xmax": 10.0,
                    "ylim_auto": True,
                    "ymin": -1.0,
                    "ymax": 1.0,
                    "show_legend": True,
                    "legend_loc": "upper right",
                    "line_colors": {} # e.g., {"q_actual": "#ff0000"}
                }
            },
            {
                "box": self.ui.lat_box, 
                "list": self.ui.list_p2, 
                "combo": self.ui.combo_add_state_2, 
                "btn": self.ui.add_p2, 
                "axis_idx": 1, 
                "defaults": ['p_actual', 'p_cmd', 'beta_actual', 'beta_cmd'],
                "settings": {
                    "grid": True,
                    "custom_label": "",
                    "xlim_auto": True,
                    "xmin": 0.0,
                    "xmax": 10.0,
                    "ylim_auto": True,
                    "ymin": -1.0,
                    "ymax": 1.0,
                    "show_legend": True,
                    "legend_loc": "upper right",
                    "line_colors": {} # e.g., {"q_actual": "#ff0000"}
                }
            },
            {
                "box": self.ui.vel_box, 
                "list": self.ui.list_p3, 
                "combo": self.ui.combo_add_state_3, 
                "btn": self.ui.add_p3, 
                "axis_idx": 2, 
                "defaults": ['V_actual', 'V_cmd'],
                "settings": {
                    "grid": True,
                    "custom_label": "",
                    "xlim_auto": True,
                    "xmin": 0.0,
                    "xmax": 10.0,
                    "ylim_auto": True,
                    "ymin": -1.0,
                    "ymax": 1.0,
                    "show_legend": True,
                    "legend_loc": "upper right",
                    "line_colors": {} # e.g., {"q_actual": "#ff0000"}
                }
            },
            {
                "box": self.ui.control_box, 
                "list": self.ui.list_p4, 
                "combo": self.ui.combo_add_state_4, 
                "btn": self.ui.add_p4, 
                "axis_idx": 3, 
                "defaults": ['throttle_actual', 'elevator_actual', 'aileron_actual', 'rudder_actual'],
                "settings": {
                    "grid": True,
                    "custom_label": "",
                    "xlim_auto": True,
                    "xmin": 0.0,
                    "xmax": 10.0,
                    "ylim_auto": True,
                    "ymin": -1.0,
                    "ymax": 1.0,
                    "show_legend": True,
                    "legend_loc": "upper right",
                    "line_colors": {} # e.g., {"q_actual": "#ff0000"}
                }
            }
        ]

        self.ui.btn_load.clicked.connect(lambda checked=False: self.load_csv(None))
        self.ui.save_plots.clicked.connect(self.save_plots_workflow)
        
        for config in self.plot_configs:
            config["list"].itemSelectionChanged.connect(self.replot)
            config["btn"].clicked.connect(lambda _, c=config: self.add_state_from_combo(c))
            
            # Search for settings buttons
            idx_num = config["axis_idx"] + 1
            btn_string_name = f"settings_p_{idx_num}"
            
            from PySide6.QtWidgets import QPushButton
            settings_button_widget = self.ui.findChild(QPushButton, btn_string_name)
            
            if settings_button_widget is not None:
                settings_button_widget.clicked.connect(lambda _, c=config: self.open_settings_popup(c))
            else:
                print(f"Warning: Could not find button named '{btn_string_name}' in UI.")
        
        for chk_name in ["display_p2", "display_p4"]: # Direct attributes
            if hasattr(self.ui, chk_name):
                getattr(self.ui, chk_name).toggled.connect(self.replot)
                
        # Fallback search for the ones inside dynamically nested layouts
        for chk_name in ["display_p1", "display_p3"]:
            chk_widget = self.ui.findChild(QWidget, chk_name)
            if chk_widget:
                chk_widget.toggled.connect(self.replot)

        if initial_file_path and os.path.exists(initial_file_path):
            self.load_csv(target_file_path=initial_file_path)

    def add_state_from_combo(self, config):
        """Toggles the highlighted item from the combo box into or out of the list widget."""
        selected_text = config["combo"].currentText()
        list_widget = config["list"]
        
        # Look for the item inside the active list tracking widget
        existing_items = list_widget.findItems(selected_text, Qt.MatchFlag.MatchExactly)
        
        if existing_items:
            # If it's already there, remove it from the tracking list view
            for item in existing_items:
                row = list_widget.row(item)
                list_widget.takeItem(row)
        else:
            # Otherwise, add it and automatically select it
            list_widget.addItem(selected_text)
            new_items = list_widget.findItems(selected_text, Qt.MatchFlag.MatchExactly)
            if new_items:
                new_items[0].setSelected(True)
        
        self.replot()

    def load_csv(self, target_file_path=None):
        if target_file_path is None or isinstance(target_file_path, bool):
            default_dir = str(Path(__file__).resolve().parent / "telemetry logs")
            file_path, _ = QFileDialog.getOpenFileName(
                self.ui, "Open Telemetry Log", default_dir, "CSV Files (*.csv)"
            )
            if not file_path:
                return
        else:
            file_path = target_file_path

        # Update the UI label name
        self.ui.lbl_file.setText(os.path.basename(file_path))
        self.data_dict.clear()

        default_dir = str(Path(__file__).resolve().parent / "telemetry logs")
        file_path, _ = QFileDialog.getOpenFileName(
            self.ui, "Open Telemetry Log", default_dir, "CSV Files (*.csv)"
        )
        if not file_path:
            return

        self.ui.lbl_file.setText(os.path.basename(file_path))
        self.data_dict.clear()

        with open(file_path, mode='r') as f:
            reader = csv.reader(f)
            self.headers = next(reader)
            temp_grid = [[] for _ in self.headers]
            for row in reader:
                if not row: continue
                for idx, val in enumerate(row):
                    temp_grid[idx].append(float(val))

        for idx, name in enumerate(self.headers):
            self.data_dict[name] = np.array(temp_grid[idx])

        if 'time' in self.data_dict:
            self.time_vec = self.data_dict['time']
            plot_headers = [h for h in self.headers if h != 'time']
        else:
            return

        # Load available options to both your scrolling fields and drop-down selectors
        for config in self.plot_configs:
            # Populate the Dropdown Menu Options
            config["combo"].blockSignals(True)
            config["combo"].clear()
            config["combo"].addItems(plot_headers)
            config["combo"].blockSignals(False)

            # Populate the Scrolling Visible List with your defaults out-of-the-box
            list_widget = config["list"]
            list_widget.blockSignals(True)
            list_widget.clear()
            list_widget.addItems(config["defaults"]) # Only show your preferred defaults on start
            list_widget.blockSignals(False)
            
            # Highlight all defaults so they load on the axes immediately
            self.set_default_selection(list_widget, config["defaults"])

        self.replot()

    def set_default_selection(self, list_widget, target_keys):
        """Helper method to programmatically highlight rows in a QListWidget."""
        list_widget.blockSignals(True)
        for key in target_keys:
            items = list_widget.findItems(key, Qt.MatchFlag.MatchExactly)
            for item in items:
                item.setSelected(True)
        list_widget.blockSignals(False)

    def replot(self):
        """Renders vectors dynamically out onto an automatically sizing grid of subplots."""
        if self.time_vec is None:
            return

        # Clear the entire figure canvas to prepare for a fresh structural layout
        self.fig.clear()

        # Map your designer checkbox object names
        checkbox_mapping = [
            self.ui.findChild(QWidget, "display_p1"),
            self.ui.display_p2,
            self.ui.findChild(QWidget, "display_p3"),
            self.ui.display_p4
        ]

        # Filter and locate configs that have active "Display plot" visibility checked
        active_configs = []
        for idx, config in enumerate(self.plot_configs):
            chk = checkbox_mapping[idx]
            if chk is None or chk.isChecked():
                active_configs.append(config)

        num_active = len(active_configs)
        if num_active == 0:
            self.canvas.draw()
            return

        # Dynamically build exactly the number of subplots needed, linking their X-axes
        axs = self.fig.subplots(num_active, 1, sharex=True)
        
        # Ensure 'axs' is always an indexable list-like array, even if there is only 1 active plot
        if num_active == 1:
            axs = [axs]

        # Loop over and render only the visible tracking tracks
        for ax_idx, config in enumerate(active_configs):
            ax = axs[ax_idx]
            ax.grid(True, linestyle=':', alpha=0.6)
            
            s = config["settings"] # Grab the saved custom properties dict
            list_widget = config["list"]
            item_count = list_widget.count()
            
            if item_count == 0:
                ax.set_ylabel("Empty Viewport")
                continue

            labels = []
            for i in range(item_count):
                item = list_widget.item(i)
                label = item.text()
                labels.append(label)
                
                if label not in self.data_dict:
                    continue
                    
                y_data = self.data_dict[label]
                display_label = label

                # Dynamics conversions
                if any(label.startswith(p) for p in ['p_', 'q_', 'r_']):
                    y_data = np.degrees(y_data)
                    display_label = f"{label} (deg/s)"
                elif any(p in label for p in ['alpha_', 'beta_']):
                    y_data = np.degrees(y_data)
                    display_label = f"{label} (deg)"
                elif any(s in label for s in ['throttle', 'elevator', 'aileron', 'rudder']):
                    display_label = f"{label} (norm)" if 'throttle' in label else f"{label} (pos)"

                style = '--' if '_cmd' in label or '_target' in label else '-'
                
                # --- THE COLOR FIX: Check for a custom saved color rule ---
                if label in s["line_colors"]:
                    ax.plot(self.time_vec, y_data, label=display_label, linestyle=style, 
                            linewidth=1.5, color=s["line_colors"][label])
                else:
                    # Let Matplotlib fall back to default cycle colors if unassigned
                    ax.plot(self.time_vec, y_data, label=display_label, linestyle=style, linewidth=1.5)

            # ax.set_ylabel(", ".join(labels[:2]) + ("..." if len(labels) > 2 else ""))
            ax.set_ylabel("")

            # Plot limit scaling
            if not s["xlim_auto"]:
                ax.set_xlim(s["xmin"], s["xmax"])
            if not s["ylim_auto"]:
                ax.set_ylim(s["ymin"], s["ymax"])

            # Legend options
            if s["show_legend"]:
                ax.legend(loc=s["legend_loc"], fontsize=8)

        # Apply the timeline label to the bottom-most active plot axis
        axs[-1].set_xlabel("Time (seconds)")
        
        # Cleanly automatically pack layout elements tightly together
        self.fig.tight_layout()
        self.canvas.draw()

    def save_plots_workflow(self):
        """Walks the user through format and quality questions, then exports the canvas."""
        if self.time_vec is None:
            return

        # Question 1: Image Format Selection
        formats = [".png", ".jpg", ".pdf", ".svg"]
        fmt, ok_fmt = QInputDialog.getItem(
            self.ui, "Export Format", "Select image file extension:", formats, 0, False
        )
        if not ok_fmt:
            return

        # Question 2: Quality/Resolution selection (DPI)
        dpi_choices = [100, 150, 200, 300, 600]
        dpi, ok_dpi = QInputDialog.getInt(
            self.ui, "Export Quality", "Select resolution quality (DPI):", 200, 100, 600, 50
        )
        if not ok_dpi:
            return

        BASE_DIR = Path(__file__).resolve().parent
        output_dir = BASE_DIR / "telemetry exports"
        output_dir.mkdir(exist_ok=True) # Safely generate folder if missing on disk

        default_save_target = str(output_dir / f"telemetry_export{fmt}")

        file_path, _ = QFileDialog.getSaveFileName(
            self.ui, "Save Figure As", default_save_target, f"Images (*{fmt})"
        )
        
        if file_path:
            # Tell the Matplotlib figure canvas object to save itself directly to disk
            self.fig.savefig(file_path, dpi=dpi, bbox_inches='tight')
    
    def open_settings_popup(self, config):
        """Launches a comprehensive properties dialog tailored to the specified subplot channel."""
        from PySide6.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QFormLayout, 
                                       QLineEdit, QDialogButtonBox, QCheckBox, QComboBox, 
                                       QGroupBox, QDoubleSpinBox, QLabel, QPushButton, QColorDialog)
        from PySide6.QtGui import QColor

        s = config["settings"] # Reference the active settings dict for brevity

        dialog = QDialog(self.ui)
        dialog.setWindowTitle(f"Plot Settings - Subplot {config['axis_idx'] + 1}")
        dialog.setMinimumWidth(380)
        
        main_layout = QVBoxLayout(dialog)
        form_layout = QFormLayout()

        # --- SECTION A: LEGEND CONTROLS ---
        legend_group = QGroupBox("Legend Settings", dialog)
        leg_form = QFormLayout(legend_group)
        
        chk_legend = QCheckBox(legend_group)
        chk_legend.setChecked(s["show_legend"])
        leg_form.addRow("Display Legend:", chk_legend)
        
        combo_leg_loc = QComboBox(legend_group)
        loc_options = ["upper right", "upper left", "lower left", "lower right", "center", "best"]
        combo_leg_loc.addItems(loc_options)
        combo_leg_loc.setCurrentText(s["legend_loc"])
        combo_leg_loc.setEnabled(s["show_legend"])
        chk_legend.toggled.connect(combo_leg_loc.setEnabled)
        leg_form.addRow("Legend Location:", combo_leg_loc)
        
        form_layout.addRow(legend_group)

        # --- SECTION B: AXIS LIMIT CONTROLS ---
        axis_group = QGroupBox("Axis Scaling Limits", dialog)
        axis_form = QFormLayout(axis_group)
        
        # X Axis Bounds
        chk_x_auto = QCheckBox(axis_group)
        chk_x_auto.setChecked(s["xlim_auto"])
        axis_form.addRow("Auto-scale X Axis:", chk_x_auto)
        
        h_layout_x = QHBoxLayout()
        spin_xmin = QDoubleSpinBox(axis_group)
        spin_xmin.setRange(-99999, 99999)
        spin_xmin.setValue(s["xmin"])
        spin_xmax = QDoubleSpinBox(axis_group)
        spin_xmax.setRange(-99999, 99999)
        spin_xmax.setValue(s["xmax"])
        h_layout_x.addWidget(QLabel("Min:"))
        h_layout_x.addWidget(spin_xmin)
        h_layout_x.addWidget(QLabel("Max:"))
        h_layout_x.addWidget(spin_xmax)
        
        # Disable spinboxes if auto-scale is on
        spin_xmin.setDisabled(s["xlim_auto"])
        spin_xmax.setDisabled(s["xlim_auto"])
        chk_x_auto.toggled.connect(lambda checked: (spin_xmin.setDisabled(checked), spin_xmax.setDisabled(checked)))
        axis_form.addRow("Manual X Range:", h_layout_x)

        # Y Axis Bounds
        chk_y_auto = QCheckBox(axis_group)
        chk_y_auto.setChecked(s["ylim_auto"])
        axis_form.addRow("Auto-scale Y Axis:", chk_y_auto)
        
        h_layout_y = QHBoxLayout()
        spin_ymin = QDoubleSpinBox(axis_group)
        spin_ymin.setRange(-99999, 99999)
        spin_ymin.setValue(s["ymin"])
        spin_ymax = QDoubleSpinBox(axis_group)
        spin_ymax.setRange(-99999, 99999)
        spin_ymax.setValue(s["ymax"])
        h_layout_y.addWidget(QLabel("Min:"))
        h_layout_y.addWidget(spin_ymin)
        h_layout_y.addWidget(QLabel("Max:"))
        h_layout_y.addWidget(spin_ymax)
        
        spin_ymin.setDisabled(s["ylim_auto"])
        spin_ymax.setDisabled(s["ylim_auto"])
        chk_y_auto.toggled.connect(lambda checked: (spin_ymin.setDisabled(checked), spin_ymax.setDisabled(checked)))
        axis_form.addRow("Manual Y Range:", h_layout_y)
        
        form_layout.addRow(axis_group)

        # --- SECTION C: DYNAMIC STATE COLOR PICKERS ---
        color_group = QGroupBox("Signal Line Colors", dialog)
        color_form = QFormLayout(color_group)
        
        # Read the current items populated in this section's tracking list
        list_widget = config["list"]
        active_signals = [list_widget.item(i).text() for i in range(list_widget.count())]
        
        # Dictionary to stash temporary color button hex overrides
        chosen_colors = s["line_colors"].copy()
        
        for signal in active_signals:
            h_box = QHBoxLayout()
            color_btn = QPushButton(dialog)
            color_btn.setFixedWidth(60)
            
            # Fetch existing saved color rule, or fall back to gray if blank
            current_hex = chosen_colors.get(signal, "#7f7f7f")
            color_btn.setStyleSheet(f"background-color: {current_hex}; border: 1px solid black;")
            
            # Sub-function to open standard operating system color picker palette
            def make_color_callback(sig=signal, btn=color_btn, init_hex=current_hex):
                def pick_color():
                    color = QColorDialog.getColor(QColor(init_hex), dialog, f"Line Color for {sig}")
                    if color.isValid():
                        hex_str = color.name()
                        btn.setStyleSheet(f"background-color: {hex_str}; border: 1px solid black;")
                        chosen_colors[sig] = hex_str
                return pick_color
                
            color_btn.clicked.connect(make_color_callback())
            h_box.addWidget(color_btn)
            h_box.addStretch()
            color_form.addRow(f"{signal}:", h_box)
            
        if active_signals:
            form_layout.addRow(color_group)

        main_layout.addLayout(form_layout)

        # Standard dialog action tray
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, dialog)
        main_layout.addWidget(buttons)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        
        # Save values if "OK" is pressed
        if dialog.exec() == QDialog.Accepted:
            s["show_legend"] = chk_legend.isChecked()
            s["legend_loc"] = combo_leg_loc.currentText()
            s["xlim_auto"] = chk_x_auto.isChecked()
            s["xmin"] = spin_xmin.value()
            s["xmax"] = spin_xmax.value()
            s["ylim_auto"] = chk_y_auto.isChecked()
            s["ymin"] = spin_ymin.value()
            s["ymax"] = spin_ymax.value()
            s["line_colors"] = chosen_colors
            
            # Redraw chart canvas to immediately reveal user alterations
            self.replot()

    def show(self):
        self.ui.show()

def main():
    app = QApplication(sys.argv)
    visualizer = TelemetryVisualizer()
    visualizer.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
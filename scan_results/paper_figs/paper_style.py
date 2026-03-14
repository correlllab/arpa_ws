"""Shared matplotlib style for IEEE CASE paper figures."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

IEEE_COL_W = 3.5    # inches, single column
IEEE_TEXT_W = 7.16   # inches, full text width (both columns)

def apply_style():
    plt.rcParams.update({
        'font.family': 'serif',
        'font.serif': ['Times', 'Times New Roman', 'DejaVu Serif'],
        'font.size': 8,
        'axes.labelsize': 9,
        'axes.titlesize': 9,
        'xtick.labelsize': 7,
        'ytick.labelsize': 7,
        'legend.fontsize': 7,
        'figure.dpi': 300,
        'savefig.dpi': 300,
        'savefig.bbox': 'tight',
        'savefig.pad_inches': 0.02,
        'lines.linewidth': 0.8,
        'axes.linewidth': 0.5,
        'grid.linewidth': 0.3,
    })

COST_CASES = [
    'baseline', 'area_only', 'joint_only', 'proximity_only',
    'no_area', 'no_joint', 'no_proximity', 'all_equal'
]

COST_LABELS = [
    'Base', 'Area', 'Joint', 'Prox',
    '~Area', '~Joint', '~Prox', 'Equal'
]

DATA_DIR = '/home/the2xman/arpa_ws/scan_results/manipulatability'
SCAN_DIR = '/home/the2xman/arpa_ws/scan_results'
OUT_DIR  = '/home/the2xman/arpa_ws/scan_results/paper_figs'

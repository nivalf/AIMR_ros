# metrics_utils.py

import psutil
import numpy as np
import os

def capture_performance_metrics():
    """
    Capture system performance metrics optimized for dashboard visualization.

    Returns:
        dict: Refined performance metrics suitable for dashboard visualization.
    """
    
    # Simplified CPU Metrics
    overall_cpu_usage = psutil.cpu_percent(interval=1)
    per_core_cpu_usages = psutil.cpu_percent(interval=1, percpu=True)
    avg_cpu_usage = np.mean(per_core_cpu_usages)
    min_cpu_usage = np.min(per_core_cpu_usages)
    max_cpu_usage = np.max(per_core_cpu_usages)
    load_avg = os.getloadavg()  # Requires 'import os'

    # Memory Metrics in GB
    memory_info = psutil.virtual_memory()
    total_memory_gb = memory_info.total / (1024 ** 3)
    available_memory_gb = memory_info.available / (1024 ** 3)
    used_memory_gb = memory_info.used / (1024 ** 3)
    
    # Swap Memory Details in GB
    swap_info = psutil.swap_memory()
    used_swap_gb = swap_info.used / (1024 ** 3)
    percent_swap = swap_info.percent

    return {
        # CPU metrics
        "overall_cpu_usage": overall_cpu_usage,
        "avg_cpu_usage": avg_cpu_usage,
        "min_cpu_usage": min_cpu_usage,
        "max_cpu_usage": max_cpu_usage,
        "load_avg_1min": load_avg[0],

        # Memory metrics
        "total_memory_gb": total_memory_gb,
        "available_memory_gb": available_memory_gb,
        "used_memory_gb": used_memory_gb,
        
        # Swap Memory metrics
        "used_swap_gb": used_swap_gb,
        "percent_swap": percent_swap
    }

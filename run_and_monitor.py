import time
import threading
import subprocess
import psutil
import pynvml
import matplotlib.pyplot as plt
import os

def monitor_resources(stop_event, data):
    pynvml.nvmlInit()
    handle = pynvml.nvmlDeviceGetHandleByIndex(0)
    
    while not stop_event.is_set():
        # CPU & RAM
        cpu_usage = psutil.cpu_percent(interval=None)
        mem = psutil.virtual_memory()
        ram_usage = mem.percent
        
        # GPU
        try:
            util = pynvml.nvmlDeviceGetUtilizationRates(handle)
            gpu_usage = util.gpu
            mem_info = pynvml.nvmlDeviceGetMemoryInfo(handle)
            vram_usage = (mem_info.used / mem_info.total) * 100
        except Exception:
            gpu_usage = 0.0
            vram_usage = 0.0

        data['time'].append(time.time())
        data['cpu'].append(cpu_usage)
        data['ram'].append(ram_usage)
        data['gpu'].append(gpu_usage)
        data['vram'].append(vram_usage)
        
        time.sleep(0.1)
    
    pynvml.nvmlShutdown()

def main():
    # Make sure output directory exists
    os.makedirs("outputs", exist_ok=True)
    
    # Initialize data dictionary
    data = {'time': [], 'cpu': [], 'ram': [], 'gpu': [], 'vram': []}
    stop_event = threading.Event()
    
    # Start monitor
    monitor_thread = threading.Thread(target=monitor_resources, args=(stop_event, data))
    monitor_thread.start()
    
    print("Running simulations with CUDA...")
    start_time = time.time()
    
    # Run the simulation for all scenarios with cuda
    cmd = '"f:/Personal Projects/SIH 2026/venv/Scripts/python.exe" -m sih_sim.cli --scenario all --device cuda --plot'
    subprocess.run(cmd, shell=True)
    
    print("Simulation complete. Stopping monitor...")
    stop_event.set()
    monitor_thread.join()
    
    # Normalize time
    if data['time']:
        t0 = data['time'][0]
        times = [t - t0 for t in data['time']]
    else:
        times = []
        
    print("Generating resource usage plot...")
    plt.figure(figsize=(10, 6))
    plt.plot(times, data['cpu'], label='CPU Usage (%)', color='blue')
    plt.plot(times, data['ram'], label='RAM Usage (%)', color='cyan')
    plt.plot(times, data['gpu'], label='GPU Usage (%)', color='red')
    plt.plot(times, data['vram'], label='VRAM Usage (%)', color='orange')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Usage (%)')
    plt.title('System Resource Usage During Simulation')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('outputs/resource_usage.png', dpi=150)
    plt.close()
    
    print("Creating collage...")
    subprocess.run('"{}" make_collage.py'.format('f:/Personal Projects/SIH 2026/venv/Scripts/python.exe'), shell=True)
    print("Done!")

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import subprocess
import json
import os
import time
import csv
import signal
from datetime import datetime

# =====================================================================
# ENME 643: STEEPEST GRADIENT DESCENT WITH AUGMENTED COST
# Optimizing a Hybrid APF + VDPF (Variable-Direction) + PID Controller
# =====================================================================
MAX_ITERATIONS = 50
TIMEOUT = 80    

# --- 1. DECISION VECTOR (X) ---
# X = [alpha_att, alpha_rep, zeta, ki, kd]
# Warm-started using optimal kinematics from your prior experiment
X_INIT = [11.30, 20.0, 0.65, 1.02, 1.75] 

# --- 2. OPTIMIZATION HYPERPARAMETERS ---
# Finite Difference Perturbations (delta)
DELTAS = [0.5, 0.5, 0.05, 0.01, 0.1]            

# Independent Base Learning Rates (gamma)
LEARNING_RATES = [0.05, 0.1, 0.01, 0.0005, 0.01] 
LR_DECAY = 0.98 

# Physical bounds [min, max]
BOUNDS = [
    (0.1, 50.0),  # alpha_att
    (0.1, 50.0),  # alpha_rep
    (0.0, 1.0),   # zeta (MUST be between 0.0 and 1.0)
    (0.0, 5.0),   # ki
    (0.0, 10.0)   # kd
]

# --- 3. LAGRANGIAN CONSTRAINT PARAMETERS ---
MIN_SAFE_DIST = 0.08   # 8cm absolute minimum safe distance
MU_PENALTY = 2000.0    # Lagrange multiplier (Mu) for constraint violation

def run_simulation(X_params):
    """Runs the simulation using the 5 parameters and securely handles the process."""
    if os.path.exists('/tmp/sim_results.json'):
        os.remove('/tmp/sim_results.json')
    
    cmd = (f"ros2 launch apf_reactive_control start_servo.launch.py "
           f"alpha_att:={X_params[0]:.3f} alpha_rep:={X_params[1]:.3f} "
           f"zeta:={X_params[2]:.3f} ki:={X_params[3]:.3f} kd:={X_params[4]:.3f}")
    
    try:
        process = subprocess.Popen(
            cmd, shell=True, preexec_fn=os.setsid, 
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        process.communicate(timeout=TIMEOUT)
    except subprocess.TimeoutExpired:
        try: os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        except ProcessLookupError: pass
        time.sleep(5) 
        
    if os.path.exists('/tmp/sim_results.json'):
        with open('/tmp/sim_results.json', 'r') as f:
            return json.load(f)
    else:
        return {"time": 60.0, "min_dist": 0.0, "jitter": 0.0, "success": False}

def augmented_cost_function(results):
    """Calculates J_a(X) = J(X) + Mu * max(0, g(X))^2"""
    time_taken = results['time']
    success = results['success']
    jitter = results.get('jitter', 0.0) 
    d_min = results.get('min_dist', 100.0)
    
    if success:
        J_base = time_taken + (0.5 * jitter)
    else:
        J_base = 260.0 # Heavy penalty for timeout or failure
        
    # Inequality Constraint: g(X) <= 0
    g_x = MIN_SAFE_DIST - d_min
    
    # Augmented Penalty
    constraint_violation = max(0.0, g_x)
    penalty = MU_PENALTY * (constraint_violation ** 2)
    
    J_augmented = J_base + penalty
    return J_augmented, J_base, penalty, d_min

def clip_value(val, min_val, max_val):
    return max(min_val, min(val, max_val))

def main():
    print("==========================================================")
    print(" ENME 643: 5-Parameter Gradient Descent w/ Augmented Cost")
    print("==========================================================")
    
    X = list(X_INIT)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    download_path = os.path.join(os.path.expanduser("~"), "Downloads", f"grad_descent_obs_{timestamp}.csv")
    
    with open(download_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Iteration', 'Alpha_Att', 'Alpha_Rep', 'Zeta', 'Ki', 'Kd', 
                         'J_Aug', 'J_Base', 'Penalty', 'Min_Dist', 
                         'Grad_Att', 'Grad_Rep', 'Grad_Zeta', 'Grad_Ki', 'Grad_Kd'])
        
        for i in range(MAX_ITERATIONS):
            current_gamma = [lr * (LR_DECAY**i) for lr in LEARNING_RATES]
            
            print(f"\n--- Iteration {i} | X = [{X[0]:.2f}, {X[1]:.2f}, {X[2]:.3f}, {X[3]:.3f}, {X[4]:.2f}] ---")
            
            base_res = run_simulation(X)
            J_aug, J_base, penalty, d_min = augmented_cost_function(base_res)
            print(f"  Cost: {J_aug:.2f} (Base: {J_base:.2f}, Penalty: {penalty:.2f}, Dist: {d_min:.3f}m)")

            gradients = [0.0] * 5
            for dim in range(5):
                X_perturbed = list(X)
                X_perturbed[dim] += DELTAS[dim]
                
                res_perturbed = run_simulation(X_perturbed)
                J_perturbed = augmented_cost_function(res_perturbed)[0]
                
                raw_gradient = (J_perturbed - J_aug) / DELTAS[dim]
                gradients[dim] = max(-50.0, min(50.0, raw_gradient))
            
            writer.writerow([i, *X, J_aug, J_base, penalty, d_min, *gradients])
            file.flush()

            for dim in range(5):
                new_val = X[dim] - (current_gamma[dim] * gradients[dim])
                X[dim] = clip_value(new_val, BOUNDS[dim][0], BOUNDS[dim][1])
            
            grad_magnitude = sum(g**2 for g in gradients)**0.5
            if grad_magnitude < 0.1:
                print("\n🛑 CONVERGED: Gradient magnitude at stationary point.")
                break

    print("\n✅ Optimization Complete.")

if __name__ == "__main__":
    main()
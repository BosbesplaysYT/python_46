import pygame
import time

def main():
    pygame.init()
    pygame.joystick.init()
    
    print("Detecting joysticks...")
    joystick_count = pygame.joystick.get_count()
    
    if joystick_count == 0:
        print("No joysticks found!")
        return
    
    # Initialize the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    
    name = joystick.get_name()
    axes = joystick.get_numaxes()
    
    print(f"\nHigh-precision monitoring: {name}")
    print(f"Axes: {axes}")
    print("Press Ctrl+C to exit...\n")
    
    # Initialize state trackers
    axis_states = [0.0] * axes
    last_print_time = time.time()
    
    try:
        while True:
            pygame.event.pump()
            current_time = time.time()
            
            # Get all current axis values with high precision
            current_values = [joystick.get_axis(i) for i in range(axes)]
            
            # Check if any values have changed
            changed = False
            for i in range(axes):
                if abs(current_values[i] - axis_states[i]) > 0.0001:
                    changed = True
                    axis_states[i] = current_values[i]
            
            # Print all values if changed or every second
            if changed or (current_time - last_print_time) > 1.0:
                # Print with 6 decimal places
                print(f"[{time.strftime('%H:%M:%S')}] Axes: {[f'{v:.6f}' for v in current_values]}")
                last_print_time = current_time
            
            time.sleep(0.01)  # Very fast update rate
            
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        pygame.quit()

if __name__ == "__main__":
    main()
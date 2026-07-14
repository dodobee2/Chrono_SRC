from src.chrono.smoke_scenario import run_smoke_scenario
r = run_smoke_scenario()
print('status=' + str(r.status))
for k in ['initial_z','minimum_z','final_z','final_vz','max_contact_count','first_contact_time_s','contact_detection_source','contact_detected','wall_time_s','max_speed_mps']:
    print(f'{k}={r.metrics.get(k)}')

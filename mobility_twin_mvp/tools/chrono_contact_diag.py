from src.chrono.smoke_scenario import run_smoke_scenario
result = run_smoke_scenario()
print('status', result.status)
print('runner_log', result.runner_log)
for key in ['initial_z','minimum_z','final_z','final_vz','max_contact_count','first_contact_time_s','contact_detection_source','contact_detected','wall_time_s']:
    print(key, result.metrics.get(key))
print('last trajectory', result.trajectory[-5:])
import pychrono as chrono
system = chrono.ChSystemNSC() if hasattr(chrono, 'ChSystemNSC') else chrono.ChSystemSMC()
print('system class', type(system))
print('has system.GetNumContacts', hasattr(system, 'GetNumContacts'))
if hasattr(system, 'GetNumContacts'):
    print('system.GetNumContacts attr', system.GetNumContacts)
print('has GetContactContainer', hasattr(system, 'GetContactContainer'))
if hasattr(system, 'GetContactContainer'):
    cc = system.GetContactContainer()
    print('container class', type(cc))
    print('has container.GetNumContacts', hasattr(cc, 'GetNumContacts'))
    names = [name for name in dir(cc) if 'Contact' in name or 'contact' in name or 'Num' in name]
    print('container names', names[:80])

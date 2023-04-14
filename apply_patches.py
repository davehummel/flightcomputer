from os.path import join

Import("env")

FRAMEWORK_DIR = join(env.PioPlatform().get_package_dir("framework-arduinoteensy"),"cores","teensy4","startup.c")
print(FRAMEWORK_DIR)


search = "if (++count >= 80) break;  // reboot after 8 seconds"
replace = "break;  // MODIFIED - reboot immediately"
  
with open(FRAMEWORK_DIR, 'r') as file:
  

    data = file.read()

    if data.find(search):
        data = data.replace(search, replace)
    else:
        exit()
  

with open(FRAMEWORK_DIR, 'w') as file:
    file.write(data)
  
# Printing Text replaced
print(f"Updated startup.c line {search} to {replace}")

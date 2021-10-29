Import("env", "projenv")

print("POST SCRIPT: uploader")

# access to global construction environment
#print(env)

# Dump construction environment (for debug purpose)
#print(env.Dump())

#print(env.get("UPLOADERFLAGS"))

#print(projenv)
#print(projenv.Dump())

project_dir = projenv.get("PROJECT_DIR")

#print(project_dir)

uploader_flags = env.get("UPLOADERFLAGS")

#print(uploader_flags)
#uploader_flags = [flags.replace('-d2', '-d3') for flags in uploader_flags]

uploader_flags = [flags.replace('target/stm32f4x.cfg', project_dir + '/custom/stm32f4x.cfg') for flags in uploader_flags]

#uploader_flags = [flags.replace('board/st_nucleo_l4.cfg', project_dir + '/custom/open-ocd_stm32l4.cfg') for flags in uploader_flags]

#print(uploader_flags)

env.Replace(
    UPLOADERFLAGS=uploader_flags
)
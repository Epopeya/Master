import os
Import("env")

# include toolchain paths
env.Replace(COMPILATIONDB_INCLUDE_TOOLCHAIN=True)

# override compilation DB path
print(os.path.join("$BUILD_DIR", "compile_commands.json"))
env.Replace(COMPILATIONDB_PATH=os.path.join("./", "compile_commands.json"))

-- 定义目标平台和工具链
toolchain("arm-none-eabi")
    set_kind("standalone")
    set_sdkdir("D:/Progrem/arm-gnu-toolchain-12.2.rel1")  -- 修改为你的工具链路径
    set_bindir("bin")
target_end()

-- 包含路径
add_includedirs(
    ".",
    "./ips",
    "./common"
)

-- 源文件
add_files(
    "main.c",
    "ips/*.c"
)

-- 项目配置
target("stm32f407-dtm")
    set_kind("binary")
    set_extension(".elf")
    set_toolchains("arm-none-eabi")
    
    -- 设置目标平台和CPU架构
    set_arch("cortex-m4")
    set_plat("cross")
    add_cxflags('-mcpu=cortex-m4 -mthumb -mfloat-abi=soft -Dgcc -Wall -O0  -gdwarf-2 -g -ffunction-sections -fdata-sections -fno-builtin-printf')
    add_asflags('-c -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -x assembler-with-cpp -Wa,-mimplicit-it=thumb -gdwarf-2 -g -ffunction-sections -fdata-sections -fno-builtin-printf')
    add_ldflags('-mcpu=cortex-m4 -mthumb -mfloat-abi=soft -nostartfiles -Wl,--gc-sections,-Map=dtm.map,-cref,-u,Reset_Handler',{force = true})
    add_ldflags(' -T ' .. os.scriptdir() .. '/link.lds')
    
    -- 包含路径
    add_includedirs(
        "arch/arm/cortex-m4",
        "./chips/stm32f407"
    )

    -- 源文件
    add_files(
        "chips/stm32f407/start.s"
    )
    
    -- 构建后生成bin文件
    after_build(function (target)
        os.run("D:/Progrem/arm-gnu-toolchain-12.2.rel1/bin/arm-none-eabi-objcopy -O binary %s firmware.bin", target:targetfile())
    end)
    
target_end()

-- 项目配置
target("stm32h743-dtm")
    set_kind("binary")
    set_extension(".elf")
    set_toolchains("arm-none-eabi")
    
    -- 设置目标平台和CPU架构
    set_arch("cortex-m4")
    set_plat("cross")
    add_cxflags('-mcpu=cortex-m4 -mthumb -mfloat-abi=soft -Dgcc -Wall -O0  -gdwarf-2 -g -ffunction-sections -fdata-sections -fno-builtin-printf')
    add_asflags('-c -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -x assembler-with-cpp -Wa,-mimplicit-it=thumb -gdwarf-2 -g -ffunction-sections -fdata-sections -fno-builtin-printf')
    add_ldflags('-mcpu=cortex-m4 -mthumb -mfloat-abi=soft -nostartfiles -Wl,--gc-sections,-Map=dtm.map,-cref,-u,Reset_Handler',{force = true})
    add_ldflags(' -T ' .. os.scriptdir() .. '/link.lds')
    
    -- 包含路径
    add_includedirs(
        "arch/arm/cortex-m4",
        "./chips/stm32h743"
    )

    -- 源文件
    add_files(
        "chips/stm32h743/start.s"
    )
    
    -- 构建后生成bin文件
    after_build(function (target)
        os.run("D:/Progrem/arm-gnu-toolchain-12.2.rel1/bin/arm-none-eabi-objcopy -O binary %s firmware.bin", target:targetfile())
    end)
    
target_end()

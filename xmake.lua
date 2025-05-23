-- 定义目标平台和工具链
toolchain("arm-none-eabi")
    set_kind("standalone")
    set_sdkdir("D:/Progrem/arm-gnu-toolchain-12.2.rel1")  -- 修改为你的工具链路径
    set_bindir("bin")
target_end()

-- 项目配置
target("firmware")
    set_kind("binary")
    set_extension(".elf")
    set_toolchains("arm-none-eabi")
    
    -- 设置目标平台和CPU架构
    add_defines("STM32F407xx")
    add_defines("DTM_CHIP=stm32f407")
    set_arch("arm")
    set_plat("cross")
    
    -- 编译选项
    add_cflags(
        "-mcpu=cortex-m4",
        "-mthumb",
        "-mfpu=fpv4-sp-d16",
        "-mfloat-abi=hard",
        "-ffunction-sections",
        "-fdata-sections",
        "-Os",
        "-Wall",
        "-Werror"
    )
    
    add_asflags(
        "-mcpu=cortex-m4",
        "-mthumb",
        "-mfpu=fpv4-sp-d16",
        "-mfloat-abi=hard"
    )
    
    add_ldflags(
        "-mcpu=cortex-m4",
        "-mthumb",
        "-mfpu=fpv4-sp-d16",
        "-mfloat-abi=hard",
        "-Wl,--gc-sections",
        "-specs=nano.specs",
        "-T$(projectdir)linker.ld"
    )
    
    -- 包含路径
    add_includedirs(
        "arch/arm/cortex-m4",
        ".",
        "./chips/stm32f407",
        "./ips",
        "./common"
    )
    
    -- 源文件
    add_files(
        "main.c",
        "chips/stm32f407/system.c",
        "ips/stm32_uart.c",
        "common/vectors.c",
        "chips/stm32f407/startup.S"
    )
    
    -- 构建后生成bin文件
    after_build(function (target)
        os.run("D:/Progrem/arm-gnu-toolchain-12.2.rel1/bin/arm-none-eabi-objcopy -O binary %s firmware.bin", target:targetfile())
    end)
    
    -- 烧录命令
    on_run(function (target)
        os.exec("st-flash write firmware.bin 0x08000000")
    end)
target_end()
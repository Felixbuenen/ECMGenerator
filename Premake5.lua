-- TODO: submodules + automatic build

workspace "ECMGenerator"
    configurations { "Debug", "Release", "ReleaseWithDebugInfo" }
    startproject "ECMApplication"
    architecture "x64"

-- ECMGenerator project
project "ECMGenerator"
    location "ECMGenerator"
    kind "StaticLib"
    language "C++"
    cppdialect "C++17"
    targetdir "bin/%{cfg.buildcfg}/%{prj.name}"
    objdir "bin-int/%{cfg.buildcfg}/%{prj.name}"

    files {
        "ECMGenerator/**.cpp",
        "ECMGenerator/**.h"
    }

    includedirs {
        "ECMGenerator",
        "./",
        "external/Boost/boost_1_82_0"
    }

    libdirs {
        "external/Boost/boost_1_82_0/libs"
    }

    vpaths {
        ["Header Files"] = { "ECMGenerator/**.h" },
        ["Source Files"] = { "ECMGenerator/**.cpp" }
    }

    filter "configurations:Debug"
        symbols "On"
        optimize "Off"

    filter "configurations:Release"
        symbols "Off"
        optimize "Full"

    filter "configurations:ReleaseWithDebugInfo"
        symbols "On"
        optimize "Full"

    filter {}

-- ECMAgentSimulator project
project "ECMAgentSimulator"
    location "ECMAgentSimulator"
    kind "StaticLib"
    language "C++"
    cppdialect "C++17"
    targetdir "bin/%{cfg.buildcfg}/%{prj.name}"
    objdir "bin-int/%{cfg.buildcfg}/%{prj.name}"

    files {
        "ECMAgentSimulator/**.cpp",
        "ECMAgentSimulator/**.h"
    }

    includedirs {
        "ECMAgentSimulator",
        "ECMGenerator",
        "external/Boost/boost_1_82_0"
    }

    links {
        "ECMGenerator"
    }

    vpaths {
        ["Header Files"] = { "ECMAgentSimulator/**.h" },
        ["Source Files"] = { "ECMAgentSimulator/**.cpp" }
    }

    filter "configurations:Debug"
        symbols "On"
        optimize "Off"

    filter "configurations:Release"
        symbols "Off"
        optimize "Full"

    filter "configurations:ReleaseWithDebugInfo"
        symbols "On"
        optimize "Full"

    filter {}

-- ECMApplication project
project "ECMApplication"
    location "ECMApplication"
    kind "ConsoleApp"
    language "C++"
    cppdialect "C++17"
    targetdir "bin/%{cfg.buildcfg}/%{prj.name}"
    objdir "bin-int/%{cfg.buildcfg}/%{prj.name}"

    files {
        "ECMApplication/**.cpp",
        "ECMApplication/**.h",
        "external/imgui/*.cpp"
    }

    includedirs {
        "ECMApplication",
        "ECMGenerator",
        "ECMAgentSimulator",
        "external/SDL2/include",
        "external/Boost/boost_1_82_0",
        "external/imgui"
    }

    libdirs {
        "external/SDL2/lib/x64"
    }

    vpaths {
        ["Header Files"] = { "ECMApplication/**.h" },
        ["Source Files"] = { "ECMApplication/**.cpp" }
    }

    links {
        "ECMGenerator",
        "ECMAgentSimulator",
        "SDL2"
    }

    dependson {
        "ECMGenerator",
        "ECMAgentSimulator"
    }

    postbuildcommands {
        "{COPYFILE} ../external/SDL2/lib/x64/SDL2.dll %{cfg.targetdir}"
    }

    filter "configurations:Debug"
        symbols "On"
        optimize "Off"

    filter "configurations:Release"
        symbols "Off"
        optimize "Full"

    filter "configurations:ReleaseWithDebugInfo"
        symbols "On"
        optimize "Full"

    filter {}

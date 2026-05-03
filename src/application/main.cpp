#include "core/bool_problem.h"
#include "io/io.h"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace
{
    using ember::BoolOp;

    // 命令行入口只负责组装两输入布尔任务，不扩展到表达式树或多输入场景。
    struct CliOptions
    {
        std::string lhsPath;
        std::string rhsPath;
        std::string outputPath;
        BoolOp operation = BoolOp::Intersection;
        std::optional<std::uint64_t> scale;
        std::size_t leafThreshold = 25;
    };

    void printUsage()
    {
        std::cerr
            << "Usage: kEmber --lhs <file.obj> --rhs <file.obj> "
            << "--op union|intersection|difference --out <result.obj> "
            << "[--scale <positive_integer>] [--leaf-threshold <positive_integer>]"
            << std::endl;
    }

    // CLI 数值参数统一要求正整数，避免导入尺度和阈值出现 0 或负值。
    bool parsePositiveUInt64(const std::string &token, std::uint64_t &outValue)
    {
        try
        {
            std::size_t consumed = 0;
            const unsigned long long parsed = std::stoull(token, &consumed, 10);
            if (consumed != token.size() || parsed == 0)
            {
                return false;
            }

            outValue = static_cast<std::uint64_t>(parsed);
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    // 第一版只开放当前内核已经支持的三种二元布尔运算。
    bool parseBoolOp(const std::string &token, BoolOp &outOp)
    {
        if (token == "union")
        {
            outOp = BoolOp::Union;
            return true;
        }
        if (token == "intersection")
        {
            outOp = BoolOp::Intersection;
            return true;
        }
        if (token == "difference")
        {
            outOp = BoolOp::Difference;
            return true;
        }

        return false;
    }

    // 解析最小外围 CLI：文件路径、布尔运算、共享量化尺度和叶子阈值。
    bool parseArgs(int argc, char **argv, CliOptions &outOptions)
    {
        outOptions = CliOptions();
        bool hasOperation = false;

        for (int i = 1; i < argc; ++i)
        {
            const std::string arg(argv[i]);
            if (arg == "--lhs" || arg == "--rhs" || arg == "--op" || arg == "--out" ||
                arg == "--scale" || arg == "--leaf-threshold")
            {
                if (i + 1 >= argc)
                {
                    std::cerr << "Missing value for argument: " << arg << std::endl;
                    return false;
                }

                const std::string value(argv[++i]);
                if (arg == "--lhs")
                {
                    outOptions.lhsPath = value;
                }
                else if (arg == "--rhs")
                {
                    outOptions.rhsPath = value;
                }
                else if (arg == "--out")
                {
                    outOptions.outputPath = value;
                }
                else if (arg == "--op")
                {
                    if (!parseBoolOp(value, outOptions.operation))
                    {
                        std::cerr << "Unsupported boolean operation: " << value << std::endl;
                        return false;
                    }

                    hasOperation = true;
                }
                else if (arg == "--scale")
                {
                    std::uint64_t scaleValue = 0;
                    if (!parsePositiveUInt64(value, scaleValue))
                    {
                        std::cerr << "Scale must be a positive integer." << std::endl;
                        return false;
                    }

                    outOptions.scale = scaleValue;
                }
                else if (arg == "--leaf-threshold")
                {
                    std::uint64_t thresholdValue = 0;
                    if (!parsePositiveUInt64(value, thresholdValue))
                    {
                        std::cerr << "Leaf threshold must be a positive integer." << std::endl;
                        return false;
                    }

                    outOptions.leafThreshold = static_cast<std::size_t>(thresholdValue);
                }

                continue;
            }

            std::cerr << "Unknown argument: " << arg << std::endl;
            return false;
        }

        if (outOptions.lhsPath.empty() ||
            outOptions.rhsPath.empty() ||
            outOptions.outputPath.empty() ||
            !hasOperation)
        {
            std::cerr << "Missing required arguments." << std::endl;
            return false;
        }

        return true;
    }

    const char *toString(BoolOp op)
    {
        switch (op)
        {
        case BoolOp::Union:
            return "union";
        case BoolOp::Intersection:
            return "intersection";
        case BoolOp::Difference:
            return "difference";
        }

        return "unknown";
    }
}

int main(int argc, char **argv)
{
    CliOptions options;
    if (!parseArgs(argc, argv, options))
    {
        printUsage();
        return 1;
    }

    try
    {
        // 应用层只做三件事：OBJ -> polygon soup、驱动 BoolProblem、结果写回 OBJ。
        ember::ObjMeshData lhsMesh;
        ember::ObjMeshData rhsMesh;
        std::string error;

        if (!ember::readObjMesh(options.lhsPath, lhsMesh, error))
        {
            std::cerr << error << std::endl;
            return 1;
        }
        if (!ember::readObjMesh(options.rhsPath, rhsMesh, error))
        {
            std::cerr << error << std::endl;
            return 1;
        }

        ember::QuantizeOptions quantizeOptions;
        quantizeOptions.explicitScale = options.scale;

        // 左右操作数必须进入同一个整数坐标系，否则后续精确谓词没有共同语义。
        std::uint64_t sharedScale = 0;
        if (!ember::chooseSharedScale({lhsMesh, rhsMesh}, quantizeOptions, sharedScale, error))
        {
            std::cerr << error << std::endl;
            return 1;
        }

        std::vector<ember::Polygon256> lhsPolygons;
        std::vector<ember::Polygon256> rhsPolygons;
        ember::PolygonSoupBuildOptions buildOptions;
        buildOptions.triangulateNonCoplanarFaces = true;
        if (!ember::buildPolygonSoup(lhsMesh, sharedScale, buildOptions, lhsPolygons, error))
        {
            std::cerr << error << std::endl;
            return 1;
        }
        if (!ember::buildPolygonSoup(rhsMesh, sharedScale, buildOptions, rhsPolygons, error))
        {
            std::cerr << error << std::endl;
            return 1;
        }

        ember::BoolProblem problem(options.leafThreshold);
        problem.setOperation(options.operation);
        problem.setOperands(lhsPolygons, rhsPolygons);
        problem.solve();

        // 默认直接导出 OBJ n-gon；三角化和拓扑恢复属于调用方后处理。
        std::size_t exportedFaces = 0;
        if (!ember::writePolygonSoupObj(problem.resultFragments(), options.outputPath, exportedFaces, error, sharedScale))
        {
            std::cerr << error << std::endl;
            return 1;
        }

        std::cout
            << "operation=" << toString(options.operation)
            << " lhs_input_faces=" << lhsMesh.faces.size()
            << " rhs_input_faces=" << rhsMesh.faces.size()
            << " scale=" << sharedScale
            << " lhs_polygons=" << lhsPolygons.size()
            << " rhs_polygons=" << rhsPolygons.size()
            << " result_fragments=" << problem.resultFragments().size()
            << " exported_faces=" << exportedFaces
            << std::endl;

        return 0;
    }
    catch (const std::exception &ex)
    {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
}

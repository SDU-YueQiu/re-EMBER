#include "core/bool_problem.h"

#include "algorithm/leaf_arrangement.h"
#include "algorithm/path_candidates.h"
#include "algorithm/WNV_tracing.h"
#include "core/logging.h"
#include "geometry/polygon_ops.h"

#include <sstream>
#include <stdexcept>
#include <utility>

namespace ember
{
    namespace
    {
        constexpr const char *kBoolProblemLeafScope = "BoolProblem::solveLeafArrangement";
        constexpr const char *kBoolProblemClassifyScope = "BoolProblem::classifyLeafFragmentsAndCollectResults";

        template <typename Builder>
        void logBoolDebug(const char *scope, Builder &&builder)
        {
            emitLogLazy(LogLevel::Debug, LogCategory::BoolProblem, scope, std::forward<Builder>(builder));
        }

        template <typename Builder>
        void logTracingDebug(const char *scope, Builder &&builder)
        {
            emitLogLazy(LogLevel::Debug, LogCategory::Tracing, scope, std::forward<Builder>(builder));
        }

        void logBoolError(const char *scope, const std::string &message)
        {
            emitLog(LogLevel::Error, LogCategory::BoolProblem, scope, message);
        }

        void validateLeafClassificationInput(
            const PlanePoint3i &referencePoint,
            const AABB3i &aabb,
            const std::vector<Polygon256> &leafFragments)
        {
            if (!referencePoint.hasUniqueIntersection())
            {
                throw std::runtime_error("Leaf classification received an invalid reference point.");
            }
            if (!isValidAABB(aabb))
            {
                throw std::runtime_error("Leaf classification received an invalid AABB.");
            }

            for (std::size_t fragmentIndex = 0; fragmentIndex < leafFragments.size(); ++fragmentIndex)
            {
                if (!leafFragments[fragmentIndex].isValid())
                {
                    std::ostringstream message;
                    message << "Leaf classification received an invalid leaf fragment at index "
                            << fragmentIndex
                            << ".";
                    throw std::runtime_error(message.str());
                }
            }
        }

        const char *traceStatusName(traceStatus status) noexcept
        {
            switch (status)
            {
            case SUCCESS:
                return "SUCCESS";
            case PATH_INVALID:
                return "PATH_INVALID";
            case INPUT_INVALID:
                return "INPUT_INVALID";
            case FAIL:
                return "FAIL";
            }

            return "UNKNOWN";
        }
    }

    // 对叶子节点内的每个 polygon 建立局部 BSP，并收集启用的 fragment。
    void BoolProblem::solveLeafArrangement()
    {
        leafFragments_.clear();
        classifiedFragments_.clear();
        if (discarded_ || polygons_.empty())
        {
            return;
        }

        leafFragments_ = buildLeafArrangement(polygons_);

        logBoolDebug(
            kBoolProblemLeafScope,
            [this]()
            {
                std::ostringstream message;
                message << "Leaf arrangement finished depth=" << depth_
                        << " input_polygons=" << polygons_.size()
                        << " leaf_fragments=" << leafFragments_.size()
                        << ".";
                return message.str();
            });
    }

    // 对每个 leaf fragment 追踪到严格内部点；分类失败属于不可恢复错误。
    void BoolProblem::classifyLeafFragmentsAndCollectResults()
    {
        resultFragments_.clear();
        classifiedFragments_.clear();
        if (discarded_ || leafFragments_.empty())
        {
            return;
        }

        try
        {
            validateLeafClassificationInput(reference_.point, aabb_, leafFragments_);
        }
        catch (const std::runtime_error &ex)
        {
            logBoolError(kBoolProblemClassifyScope, ex.what());
            throw;
        }

        const refPoint localReference(reference_.point, reference_.wnv);
        for (std::size_t fragmentIndex = 0; fragmentIndex < leafFragments_.size(); ++fragmentIndex)
        {
            Polygon256 &fragment = leafFragments_[fragmentIndex];
            const std::vector<PlanePoint3i> pointCandidates =
                detail::enumerateLeafClassificationPointCandidatesUnchecked(fragment);
            const std::vector<LeafClassificationPathCandidate> fastCandidates =
                enumerateLeafClassificationFastPathCandidatesFromPoints(reference_.point, pointCandidates, aabb_);
            const std::size_t fastCandidateCount = fastCandidates.size();
            std::size_t fallbackCandidateCount = 0;

            bool classified = false;
            traceStatus lastStatus = FAIL;
            auto traceCandidate =
                [&](const LeafClassificationPathCandidate &candidate, const char *candidateLayer, std::size_t candidateIndex)
                {
                    WNV frontWNV;
                    WNV backWNV;
                    const traceStatus status = tracePathWNVToSurfacePoint(
                        localReference,
                        candidate.path,
                        polygons_,
                        fragment.plane,
                        frontWNV,
                        backWNV);

                    logTracingDebug(
                        kBoolProblemClassifyScope,
                        [fragmentIndex, candidateLayer, candidateIndex, &candidate, status]()
                        {
                            std::ostringstream message;
                            message << "Leaf fragment trace attempt fragment_index=" << fragmentIndex
                                    << " candidate_layer=" << candidateLayer
                                    << " candidate_index=" << candidateIndex
                                    << " path_segments=" << candidate.path.size()
                                    << " status=" << traceStatusName(status)
                                    << ".";
                            return message.str();
                        });

                    if (status == SUCCESS)
                    {
                        classifiedFragments_.push_back(
                            ClassifiedFragment{fragment, std::move(frontWNV), std::move(backWNV)});
                        classified = true;
                    }

                    return status;
                };

            bool allowFallback = fastCandidates.empty();
            for (std::size_t candidateIndex = 0; candidateIndex < fastCandidates.size(); ++candidateIndex)
            {
                const traceStatus status = traceCandidate(fastCandidates[candidateIndex], "fast", candidateIndex);
                if (status == SUCCESS)
                {
                    break;
                }

                lastStatus = status;
                if (status != PATH_INVALID)
                {
                    allowFallback = false;
                    break;
                }
                allowFallback = true;
            }

            if (!classified && allowFallback)
            {
                enumerateLeafClassificationFallbackPathCandidatesFromPoints(
                    reference_.point,
                    pointCandidates,
                    aabb_,
                    [&](LeafClassificationPathCandidate candidate)
                    {
                        const std::size_t candidateIndex = fallbackCandidateCount;
                        ++fallbackCandidateCount;
                        const traceStatus status = traceCandidate(candidate, "fallback", candidateIndex);
                        if (status == SUCCESS)
                        {
                            return false;
                        }

                        lastStatus = status;
                        return status == PATH_INVALID;
                    });
            }

            if (!classified)
            {
                std::ostringstream message;
                message << "BoolProblem failed to classify leaf fragment "
                        << fragmentIndex
                        << " at depth "
                        << depth_
                        << " after "
                        << fastCandidateCount
                        << " fast path candidates and "
                        << fallbackCandidateCount
                        << " fallback path candidates from "
                        << pointCandidates.size()
                        << " point candidates; last trace status = "
                        << traceStatusName(lastStatus)
                        << ".";
                logBoolError(kBoolProblemClassifyScope, message.str());
                throw std::runtime_error(message.str());
            }

            const ClassifiedFragment &classifiedFragment = classifiedFragments_.back();
            const BoolStatus frontStatus = evaluateBooleanIndicator(classifiedFragment.frontWNV);
            const BoolStatus backStatus = evaluateBooleanIndicator(classifiedFragment.backWNV);
            if (frontStatus == OUT && backStatus == IN)
            {
                resultFragments_.push_back(classifiedFragment.polygon);
            }
            else if (frontStatus == IN && backStatus == OUT)
            {
                resultFragments_.push_back(reversePolygonOrientation(classifiedFragment.polygon));
            }
        }

        logBoolDebug(
            kBoolProblemClassifyScope,
            [this]()
            {
                std::ostringstream message;
                message << "Leaf classification finished depth=" << depth_
                        << " result_fragments=" << resultFragments_.size()
                        << ".";
                return message.str();
            });
    }

    // 将 WNV 交给当前布尔运算的二元指示函数，返回内外状态。
    BoolStatus BoolProblem::evaluateBooleanIndicator(const WNV &wnv) const noexcept
    {
        WNV tmp = wnv;
        switch (op_)
        {
        case BoolOp::Union:
            return f_union(tmp, 0, 1);
        case BoolOp::Intersection:
            return f_intersection(tmp, 0, 1);
        case BoolOp::Difference:
            return f_diff(tmp, 0, 1);
        }

        return OUT;
    }
}

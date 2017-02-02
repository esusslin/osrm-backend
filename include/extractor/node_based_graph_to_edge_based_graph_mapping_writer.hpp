#ifndef NODE_BASED_GRAPH_TO_EDGE_BASED_GRAPH_MAPPING_WRITER_HPP
#define NODE_BASED_GRAPH_TO_EDGE_BASED_GRAPH_MAPPING_WRITER_HPP

#include "storage/io.hpp"
#include "util/log.hpp"
#include "util/typedefs.hpp"

#include <boost/assert.hpp>

#include <cstddef>

#include <string>

namespace osrm
{
namespace extractor
{

// Writes:  | Fingerprint | #mappings | u v head tail | u v head tail | ..
// - uint64: number of mappings (u, v, head, tail) chunks
// - NodeID u, NodeID v, EdgeID head, EdgeID tail

struct NodeBasedGraphToEdgeBasedGraphMappingWriter
{
    NodeBasedGraphToEdgeBasedGraphMappingWriter(const std::string &path)
        : writer{path, storage::io::FileWriter::GenerateFingerprint}, num_written{0}
    {
        const std::uint64_t dummy{0}; // filled in later
        writer.WriteElementCount64(dummy);
    }

    void WriteMapping(NodeID u, NodeID v, EdgeID head, EdgeID tail)
    {
        BOOST_ASSERT(u != SPECIAL_NODEID);
        BOOST_ASSERT(v != SPECIAL_NODEID);
        BOOST_ASSERT(head != SPECIAL_EDGEID || tail != SPECIAL_EDGEID);

        writer.WriteOne(u);
        writer.WriteOne(v);
        writer.WriteOne(head);
        writer.WriteOne(tail);

        num_written += 1;
    }

    ~NodeBasedGraphToEdgeBasedGraphMappingWriter()
    {
        if (num_written != 0)
        {
            writer.SkipToBeginning();
            writer.WriteOne(num_written);
        }
    }

  private:
    storage::io::FileWriter writer;
    std::uint64_t num_written;
};

} // ns extractor
} // ns osrm

#endif // NODE_BASED_GRAPH_TO_EDGE_BASED_GRAPH_MAPPING_WRITER_HPP

# OpenCog Collection (OCC) - AGI Operating System

**Version**: 3.0.0  
**Repository**: https://github.com/o9nn/occ  
**License**: AGPL-3.0

> ### An Integrated Architecture for Cognitive Synergy and AGI-OS

**The OpenCog Collection (OCC) is a comprehensive monorepo that integrates the entire OpenCog ecosystem with operating system-level cognitive capabilities.** Our primary goal is to foster **cognitive synergy**, where the interaction of diverse AI components leads to emergent intelligence and capabilities beyond the sum of their individual parts.

This repository provides a complete, production-ready environment for research and development in Artificial General Intelligence (AGI), featuring a three-layer architecture from microkernel to cognitive applications.

---

## Key Features

### Cognitive Synergy Architecture
- **Unified Cognitive Architecture**: Coherent integration of AtomSpace hypergraph database with advanced AI reasoning, learning, and attention systems
- **Cognitive Synergy by Design**: Explicit facilitation of interaction between symbolic reasoning, machine learning, and evolutionary algorithms
- **Autogenesis - Self-Evolution**: AI-powered system identifying novel features in the "adjacent possible" through component synergy
- **Cross-Modal Cognitive Fusion**: Unified cognitive loops integrating PLN reasoning, MOSES learning, and AtomSpace memory with bidirectional feedback
- **Distributed Cognitive Shard Network**: Specialized cognitive processing units with shared memory for parallel distributed cognition

### AGI-OS Integration
- **Three-Layer AGI-OS Stack**: CogNumach microkernel → HurdCog operating system → OCC cognitive framework
- **Kernel-Level Cognitive Primitives**: Attention-aware scheduling, distributed hypergraph storage, and reasoning at the OS level
- **AtomSpace-MachSpace Bridge**: Direct integration between hypergraph knowledge representation and microkernel memory management
- **Production-Ready Packaging**: Complete Debian packaging infrastructure and Windows builds with Chocolatey support

### Development Excellence
- **Reproducible Environment**: GNU Guix declarative builds and devcontainer support for consistent development
- **Extensible and Modular**: Easy integration of new components and cognitive architectures
- **Comprehensive Testing**: Automated CI/CD with cognitive synergy validation
- **Focus on AGI Research**: Platform designed specifically for building and experimenting with AGI systems

---

## Architecture

### Three-Layer AGI-OS Stack

```
┌─────────────────────────────────────────────────────────────┐
│  Layer 3: OCC Framework (Cognitive Applications)            │
│  ├─ AtomSpace (Hypergraph Knowledge Representation)         │
│  ├─ PLN (Probabilistic Logic Networks)                      │
│  ├─ URE (Unified Rule Engine)                               │
│  ├─ ECAN (Economic Attention Networks)                      │
│  ├─ CogGML (Self-Aware Microkernel Shards)                  │
│  ├─ CogSelf (AGI Synergy Framework)                         │
│  ├─ MOSES (Evolutionary Program Learning)                   │
│  └─ Agentic Chatbots, Learning, Vision, Bioinformatics      │
├─────────────────────────────────────────────────────────────┤
│  Integration Layer (Cognitive Bridges)                      │
│  ├─ AtomSpace ↔ MachSpace Bridge                           │
│  ├─ ECAN ↔ Cognitive Scheduler                             │
│  └─ URE ↔ CogKernel Bridge                                 │
├─────────────────────────────────────────────────────────────┤
│  Layer 2: HurdCog (Cognitive Operating System)              │
│  ├─ MachSpace (Distributed Hypergraph Storage)              │
│  ├─ CogKernel (Kernel-Level Reasoning)                      │
│  ├─ Cognitive Translators                                   │
│  └─ AtomSpace-Hurd Bridge                                   │
├─────────────────────────────────────────────────────────────┤
│  Layer 1: CogNumach (Cognitive Microkernel)                 │
│  ├─ Enhanced GNU Mach Microkernel                           │
│  ├─ Cognitive Scheduler (Attention-Aware CPU Allocation)    │
│  ├─ IPC with Cognitive Primitives                           │
│  └─ Memory Management with Attention Values                 │
└─────────────────────────────────────────────────────────────┘
```

### Core Components

#### Foundation
- **cogutil** - Core C++ utilities and infrastructure
- **atomspace** - Hypergraph database for knowledge representation
- **atomspace-storage** - Persistence backends (RocksDB, PostgreSQL, CogServer)
- **atomspace-accelerator** - High-performance inference engine

#### Reasoning & Learning
- **ure** - Unified Rule Engine for forward/backward chaining
- **pln** - Probabilistic Logic Networks for uncertain reasoning
- **miner** - Pattern mining and discovery
- **learn** - Language learning and grammar induction
- **moses** - Meta-Optimizing Semantic Evolutionary Search

#### Cognitive Architecture
- **coggml** - Self-aware cognitive microkernel with shard architecture
- **cogself** - AGI cognitive synergy framework
- **attention** - Economic Attention Networks (ECAN)
- **matrix** - Sparse matrix operations on AtomSpace

#### Integration & Services
- **cogserver** - Network server for AtomSpace access
- **agents** - Interactive cognitive agents
- **sensory** - Sensory dataflow processing

#### Applications
- **agentic-chatbots** - Conversational AI with cognitive grounding
- **vision** - Computer vision integration
- **agi-bio** - Bioinformatics and genomics

---

## Quick Start

### System Dependencies

**Ubuntu/Debian**:
```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git \
  libboost-all-dev guile-3.0-dev \
  python3-dev cython3 \
  liboctomap-dev liboctomap-tools
```

**Windows**:
- Visual Studio 2019 or later
- vcpkg package manager
- CMake 3.12+

### Option 1: Quick Build (Recommended)

```bash
git clone --recurse-submodules https://github.com/o9nn/occ.git
cd occ
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

### Option 2: Build with AGI-OS Stack (Advanced)

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_COGNUMACH=ON \
    -DBUILD_HURDCOG=ON \
    -DBUILD_INTEGRATION_LAYER=ON \
    -DBUILD_COGGML=ON \
    -DBUILD_COGSELF=ON
make -j$(nproc)
sudo make install
```

### Option 3: Debian Packages

```bash
cd opencog-debian
./build-all-production.sh
```

### Option 4: Windows Build

```powershell
.\build-windows.ps1 -BuildAllComponents
```

### Option 5: Devcontainer (Recommended for Development)

1. Open repository in VS Code
2. Reopen in container when prompted
3. All dependencies automatically configured

---

## Cognitive Synergy Enhancements

### 1. Cross-Modal Cognitive Fusion ✅

Integrates reasoning, learning, and memory into unified cognitive loops:
- Shared representation layer in AtomSpace
- PLN → MOSES feedback loop (reasoning guides learning)
- MOSES → PLN feedback loop (learning enriches reasoning)
- Meta-learning system that evolves cognitive strategies

**Documentation**: `docs/cross-modal-cognitive-fusion.md`

### 2. Distributed Cognitive Shard Network ✅

Network of specialized cognitive shards with shared memory:
- 9 shard specializations (reasoning, learning, pattern mining, etc.)
- Thread-safe shared AtomSpace view
- Inter-shard communication via message bus
- Dynamic task allocation and load balancing

**Documentation**: `docs/distributed-cognitive-shard-network.md`

### 3. Attention-Guided Evolutionary Learning ✅

Uses attention allocation to guide MOSES learning:
- Extract high-STI atoms from attention system
- Convert attention signals to MOSES fitness bonuses
- Feedback loop from learning outcomes to attention

**Documentation**: `docs/attention-guided-learning.md`

### Unified Synergy Check

Run the synergy validation script to ensure all components work together:

```bash
./synergy.sh
```

This script builds all components and performs interoperability tests, validating the cognitive synergy architecture.

---

## Usage Examples

### AtomSpace Basics (Scheme)

```scheme
(use-modules (opencog))

; Create atoms
(Concept "AI")
(Concept "AGI")

; Create relationships
(Inheritance (Concept "AGI") (Concept "AI"))

; Query the AtomSpace
(cog-get-atoms 'Concept)
```

### AtomSpace Basics (Python)

```python
from opencog.atomspace import AtomSpace, types
from opencog.type_constructors import *

atomspace = AtomSpace()
ai = ConceptNode("AI")
agi = ConceptNode("AGI")
InheritanceLink(agi, ai)
```

### Using CogServer

```bash
guile -c "(use-modules (opencog cogserver)) (start-cogserver)"
telnet localhost 17001
```

---

## Documentation

### Core Documentation
- **[Architecture Overview](docs/architecture.md)** - Detailed OCC architecture
- **[Cognitive Synergy](docs/cognitive-synergy.md)** - Principles and implementation
- **[Autogenesis](docs/autogenesis.md)** - AI-powered feature generation
- **[Contributing Guide](CONTRIBUTING.md)** - How to contribute

### AGI-OS Documentation
- **[AGI Kernel Evaluation](docs/AGI_KERNEL_EVALUATION.md)** - OS kernel primitives for AGI
- **[AGI-OS Integration Guide](docs/AGI_OS_INTEGRATION_GUIDE.md)** - Technical integration specs
- **[AGI-Kern Coverage Summary](docs/AGI_KERN_COVERAGE_SUMMARY.md)** - Feature coverage reference

### Build & Infrastructure
- **[Build Guide](docs/build/)** - Comprehensive build instructions
- **[Guix SSR Implementation](docs/guix-ssr-implementation.md)** - Reproducible builds
- **[Debian Packaging](opencog-debian/README_COMPLETE.md)** - Package infrastructure

---

## Build Options

**AGI-OS Stack**:
- `BUILD_COGNUMACH` - Cognitive microkernel (OFF by default)
- `BUILD_HURDCOG` - Cognitive OS (OFF by default)
- `BUILD_INTEGRATION_LAYER` - Integration bridges (ON by default)

**Core Components**:
- `BUILD_COGUTIL` - Core utilities (ON)
- `BUILD_ATOMSPACE` - Hypergraph database (ON)
- `BUILD_ATOMSPACE_STORAGE` - Storage backends (ON)
- `BUILD_COGSERVER` - Network server (ON)

**Cognitive Architecture**:
- `BUILD_COGGML` - CogGML microkernel (ON)
- `BUILD_COGSELF` - CogSelf framework (ON)
- `BUILD_ATOMSPACE_ACCELERATOR` - Inference accelerator (ON)
- `BUILD_AGENTIC_CHATBOTS` - Chatbot integration (ON)

---

## Releases

### Version 3.0.0 (Current)
- Complete AGI-OS integration with CogNumach and HurdCog
- CogGML self-aware microkernel architecture
- CogSelf AGI synergy framework
- AtomSpace accelerator inference engine
- Complete Debian packaging (40+ packages)
- Windows build automation with Chocolatey
- Cross-modal cognitive fusion
- Distributed cognitive shard network

---

## Community and Support

- **Website**: https://opencog.org/
- **Wiki**: https://wiki.opencog.org/
- **Discussions**: [GitHub Discussions](https://github.com/opencog/occ/discussions)
- **Mailing List**: [OpenCog Google Group](https://groups.google.com/g/opencog)
- **Issues**: https://github.com/o9nn/occ/issues

---

## License

AGPL-3.0 License - see [LICENSE](LICENSE) file for details.

## Citation

```bibtex
@software{opencog2025,
  title = {OpenCog Collection: AGI Operating System},
  author = {OpenCog Foundation},
  year = {2025},
  url = {https://github.com/o9nn/occ},
  version = {3.0.0}
}
```

---

**Built with cognitive synergy for the future of AGI** 🧠✨

*The OpenCog Collection continues the pioneering work of Dr. Ben Goertzel and the OpenCog community.*

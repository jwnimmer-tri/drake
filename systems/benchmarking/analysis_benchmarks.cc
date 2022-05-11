#include <benchmark/benchmark.h>

#include "drake/systems/framework/diagram_builder.h"
#include "drake/tools/performance/fixture_common.h"

/* A collection of scenarios to benchmark, scoped to cover all code within the
drake/systems/analysis package. */

namespace drake {
namespace systems {
namespace {

class BasicFixture : public benchmark::Fixture {
 public:
  BasicFixture() {
    tools::performance::AddMinMaxStatistics(this);
  }

  // This apparently futile using statement works around "overloaded virtual"
  // errors in g++. All of this is a consequence of the weird deprecation of
  // const-ref State versions of SetUp() and TearDown() in benchmark.h.
  using benchmark::Fixture::SetUp;
  void SetUp(benchmark::State&) override {
    builder_ = std::make_unique<DiagramBuilder<double>>();
  }

  void Build() {
    diagram_ = builder_->Build();
    context_ = diagram_->CreateDefaultContext();
    builder_.reset();
  }

 protected:
  std::unique_ptr<DiagramBuilder<double>> builder_;
  std::unique_ptr<Diagram<double>> diagram_;
  std::unique_ptr<Context<double>> context_;
};

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_F(BasicFixture, MonteCarlo)(benchmark::State& state) {
  // We'd like to measure the effect of scale on a few variables:
  // - How much randomness the make_simulator consumes.
  // - How much compute the make_simulator needs.
  // - 

  const SimulatorFactory& make_simulator;
  const ScalarSystemFunction& output;

  // Warm up the random generator.
  RandomGenerator generator;
  generator();

  for (auto _ : state) {
    double final_time{};
    int num_samples{};
    int num_parallel_executions{};
    int num_threads{};
    std::vector<RandomSimulationResult> results = MonteCarloSimulation(
        make_simulator, output, final_time, num_samples, &generator, num_threads);
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake

BENCHMARK_MAIN();

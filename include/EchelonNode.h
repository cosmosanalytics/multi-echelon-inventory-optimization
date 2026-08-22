#pragma once

#include <string>

namespace meio {

// A single stage ("echelon") in a serial supply chain, ordered upstream
// (e.g. Plant) to downstream (e.g. Local DC / customer-facing node).
class EchelonNode {
public:
    EchelonNode(int id, std::string name, int leadTime, double holdingCostPerUnit)
        : id_(id),
          name_(std::move(name)),
          leadTime_(leadTime),
          holdingCostPerUnit_(holdingCostPerUnit) {}

    int id() const { return id_; }
    const std::string& name() const { return name_; }

    // This stage's own replenishment/processing lead time, in days.
    int leadTime() const { return leadTime_; }

    // Holding cost per unit of safety stock held at this stage.
    double holdingCostPerUnit() const { return holdingCostPerUnit_; }

private:
    int id_;
    std::string name_;
    int leadTime_;
    double holdingCostPerUnit_;
};

} // namespace meio

module ReassemblyControlStuff

using ControlSystems
using LinearAlgebra

global Next_Mover_Index::UInt = 0

abstract type AbstractBlock end

struct ThrusterBlock <: AbstractBlock
    id::UInt32
    thrusterForce::Float32
    orientation_vector::Vector{Float32}
    offset::Tuple{Float32,Float32}
    r::Float32
    angle::Float32
    function ThrusterBlock(id, thrusterForce::Float32, offset::Tuple{Float32,Float32}, angle::Float32)
        orientation_vector = [cos(angle), sin(angle)]
        r = norm(offset)
        new(id, thrusterForce, orientation_vector, offset, r, angle)
    end
    function ThrusterBlock(id, thrusterForce, offset::Vector{Float32}, angle)
        if length(offset) != 2
            error("`offset` must a Tuple or Vector with length 2, but is length $(length(offset))!")
        end
        new(id, thrusterForce, (offset[1], offset[2]), angle)
    end
end



struct TorquerBlock <: AbstractBlock
    id::UInt32
    torquerTorque::Float32
end

abstract type AbstractActuator end

mutable struct Thruster <: AbstractActuator
    idx::UInt
    thruster::ThrusterBlock
    u::Float32
end
mutable struct Torquer <: AbstractActuator
    idx::UInt
    torquer::TorquerBlock
    u::Float32
end
mutable struct ThrusterTorquer <: AbstractActuator
    idx::UInt
    thruster::ThrusterBlock
    torquer::TorquerBlock
    u::Float32
end

force(thruster::Thruster) = thruster.u * thruster.thruster.thrusterForce
torque(thruster::Thruster) = 0
function setInput!(mover::AbstractActuator, u)
    if !(0 ≤ u ≤ 1)
        mover.u = u
        return
    end
    uclamp = max(min(u, 1), -1)
    @debug "Input to mover clamped." u uclamp
    mover.u = uclamp
end


## implement Reassembly's current control system

greet() = print("Hello World!")

end # module ReassemblyControlStuff

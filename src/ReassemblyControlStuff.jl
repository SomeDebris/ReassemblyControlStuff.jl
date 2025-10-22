module ReassemblyControlStuff

using ControlSystems
using LinearAlgebra

global Next_Mover_Index::UInt = 0

mutable struct ShipStateGlobalReferenceFrame
    x
    y
    Vx
    Vy
    θ
    ω
end
statevect(s::ShipStateGlobalReferenceFrame) = [s.x, s.y, s.Vx, s.Vy, s.θ, s.ω]

abstract type AbstractActuatorParameters end

struct Block
    ident::UInt32
    name::String
    offset::Tuple{Float32,Float32}
    angle::Float32
    r::Float32
    orientation_vector::Vector{Float32}
    J_shape::Float32
    J_inposition::Float32
    m::Float32
end


struct Thruster <: AbstractActuatorParameters
    ident::UInt32
    thrusterForce::Float32
    orientation_vector::Vector{Float32}
    offset::Tuple{Float32,Float32}
    r::Float32
    angle::Float32
    function Thruster(ident, thrusterForce::Float32, offset::Tuple{Float32,Float32}, angle::Float32)
        orientation_vector = [cos(angle), sin(angle)]
        r = norm(offset)
        new(ident, thrusterForce, orientation_vector, offset, r, angle)
    end
    function Thruster(ident, thrusterForce, offset::Vector{Float32}, angle)
        if length(offset) != 2
            error("`offset` must a Tuple or Vector with length 2, but is length $(length(offset))!")
        end
        new(ident, thrusterForce, (offset[1], offset[2]), angle)
    end
end



struct Torquer <: AbstractActuatorParameters
    ident::UInt32
    torquerTorque::Float32
end

abstract type AbstractActuator end

mutable struct ThrusterActuator <: AbstractActuator
    index::UInt
    thruster::Thruster
    u::Float32
    enabled::Bool
end
mutable struct TorquerActuator <: AbstractActuator
    index::UInt
    torquer::Torquer
    u::Float32
    enabled::Bool
end
mutable struct ThrusterTorquerActuator <: AbstractActuator
    index::UInt
    thruster::Thruster
    torquer::Torquer
    u::Float32
    enabled::Bool
end

force(thruster::ThrusterActuator) = thruster.u * thruster.thruster.thrusterForce
torque(thruster::ThrusterActuator) = 0
function setInput!(a::AbstractActuator, u)
    if !(0 ≤ u ≤ 1)
        a.u = u
        return
    end
    uclamp = clamp(u, 0, 1)
    @debug "Input to actuator clamped." a u uclamp
    a.u = uclamp
end
function enable!(a::AbstractActuator)
    if a.enabled
        @debug "Actuator was already enabled." a
    end
    a.enabled = true
end
function disable!(a::AbstractActuator)
    if !a.enabled
        @debug "Actuator was already disabled." a
    end
    a.enabled = false
end


mutable struct ShipGlobal
    I::Float32
    m::Float32
    actuators::Vector{AbstractActuator}
    blocks::Vector{Block}
    # if block is destroyed, bool of this becomes 1
    block_destroyed_mask::Vector{Bool}
    state::ShipStateGlobalReferenceFrame
end


## implement Reassembly's current control system

greet() = print("Hello World!")

end # module ReassemblyControlStuff

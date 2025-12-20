---
sidebar_position: 6
---

# Compliant Contact Models

Compliant contact models are essential for creating realistic interactions in Gazebo simulations. Unlike rigid contact models, compliant models allow for deformation and more realistic force responses during collisions.

## Understanding Compliant Contacts

In real-world physics, objects are not perfectly rigid. When two objects make contact, they deform slightly, creating contact patches and distributed forces. Compliant contact models simulate this behavior, resulting in more stable and realistic simulations.

## Implementation in Gazebo

In Gazebo, compliant contacts can be configured using the ODE (Open Dynamics Engine) physics engine parameters:

```xml
<sdf version="1.7">
  <world name="default">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.000001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>
  </world>
</sdf>
```

### Key Parameters

- `cfm` (Constraint Force Mixing): Controls the compliance of constraints. Lower values result in stiffer constraints.
- `erp` (Error Reduction Parameter): Controls how quickly constraint errors are corrected. Higher values correct errors faster but can cause instability.
- `contact_surface_layer`: The depth of the contact layer, allowing bodies to slightly penetrate each other without generating forces.

## Surface Properties

For more realistic contact models, you can define surface properties at the link level:

```xml
<link name="wheel">
  <collision name="collision">
    <geometry>
      <sphere>
        <radius>0.1</radius>
      </sphere>
    </geometry>
    <surface>
      <contact>
        <ode>
          <soft_cfm>0.0001</soft_cfm>
          <soft_erp>0.2</soft_erp>
          <kp>100000000.0</kp>
          <kd>1.0</kd>
          <max_vel>100.0</max_vel>
          <min_depth>0.001</min_depth>
        </ode>
      </contact>
      <friction>
        <ode>
          <mu>1.0</mu>
          <mu2>1.0</mu2>
          <fdir1>0 0 1</fdir1>
        </ode>
      </friction>
    </surface>
  </collision>
</link>
```

## Best Practices

1. **Tune Parameters Carefully**: Start with conservative values and gradually adjust for your specific simulation needs.
2. **Balance Stability and Realism**: More compliant contacts are more realistic but may require smaller time steps for stability.
3. **Consider Computational Cost**: Compliant contacts can be more computationally expensive than rigid contacts.

## Integration with Unity

When synchronizing with Unity, ensure that the compliance parameters in Gazebo match the expected behavior in Unity's physics engine for consistent simulation results.
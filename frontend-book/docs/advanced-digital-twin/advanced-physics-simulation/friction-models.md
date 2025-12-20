---
sidebar_position: 7
---

# Friction Models

Friction models are crucial for realistic physics simulation in Gazebo. Advanced friction modeling includes static, dynamic, and viscous friction components that accurately represent real-world contact behaviors.

## Types of Friction

### Static Friction
Static friction prevents objects from starting to move. It's generally higher than kinetic friction and must be overcome to initiate motion.

### Dynamic (Kinetic) Friction
Dynamic friction acts on objects that are already in motion. It's typically lower than static friction and opposes the direction of motion.

### Viscous Friction
Viscous friction is velocity-dependent and represents the resistance that increases with speed, common in lubricated contacts.

## Implementation in Gazebo

Gazebo uses the ODE physics engine for friction calculations. Here's how to configure advanced friction models:

```xml
<link name="object_with_advanced_friction">
  <collision name="collision">
    <surface>
      <friction>
        <ode>
          <!-- Primary friction coefficient -->
          <mu>0.8</mu>

          <!-- Secondary friction coefficient (for 2D contact) -->
          <mu2>0.8</mu2>

          <!-- Direction of anisotropic friction -->
          <fdir1>1 0 0</fdir1>

          <!-- Slip coefficients -->
          <slip1>0.0</slip1>
          <slip2>0.0</slip2>
        </ode>

        <!-- Bullet physics friction model (alternative) -->
        <bullet>
          <friction>0.8</friction>
          <friction2>0.8</friction2>
          <fdir1>1 0 0</fdir1>
          <rolling_friction>0.01</rolling_friction>
        </bullet>
      </friction>
    </surface>
  </collision>
</link>
```

### Anisotropic Friction

Anisotropic friction varies with direction. This is useful for simulating surfaces like brushed metal or tire treads:

```xml
<surface>
  <friction>
    <ode>
      <mu>0.9</mu>      <!-- Friction in primary direction -->
      <mu2>0.3</mu2>    <!-- Friction in secondary direction -->
      <fdir1>0 1 0</fdir1>  <!-- Primary friction direction -->
    </ode>
  </friction>
</surface>
```

### Advanced Friction Parameters

- `slip1` and `slip2`: Allow for velocity-dependent friction (slip models)
- `fdir1`: Defines the direction of anisotropic friction
- `rolling_friction`: Models resistance to rolling motion (Bullet only)

## Stribeck Friction Model

For more realistic friction behavior, consider implementing a Stribeck model that transitions between static and dynamic friction:

```python
# Example Python code for custom friction controller
def calculate_friction_force(v_relative, mu_static, mu_dynamic, v_transition):
    """
    Calculate friction force using Stribeck model
    """
    if abs(v_relative) < 0.001:  # Nearly static
        return mu_static
    elif abs(v_relative) < v_transition:  # Stribeck effect
        return mu_dynamic + (mu_static - mu_dynamic) * \
               math.exp(-abs(v_relative) / v_transition)
    else:  # Pure kinetic friction
        return mu_dynamic
```

## Best Practices

1. **Tune Friction Coefficients**: Use literature values for material pairs or experimental data when available
2. **Consider Anisotropic Effects**: For directional materials like tire treads or brushed surfaces
3. **Validate Against Real Data**: Compare simulation results with physical tests when possible
4. **Balance Accuracy and Performance**: More complex friction models may impact simulation speed

## Integration with Unity

When synchronizing with Unity, ensure that friction parameters in Gazebo correspond to similar behaviors in Unity's PhysX engine for consistent simulation results across platforms.
function dU = n2o_energy_flux(m_dot_N2_in, T_hp, m_dot_L_out, h_L_now, cp_N2)
%N2O_ENERGY_FLUX Rate of change of the LP tank's total internal energy.
% Adiabatic walls: N2 enters at HP-line temperature carrying cp*T enthalpy;
% saturated liquid leaves carrying h_L. Must be fed the SAME clamped liquid
% flow that feeds the m_N2O integrator.
%#codegen
dU = m_dot_N2_in * cp_N2 * T_hp - m_dot_L_out * h_L_now;
end

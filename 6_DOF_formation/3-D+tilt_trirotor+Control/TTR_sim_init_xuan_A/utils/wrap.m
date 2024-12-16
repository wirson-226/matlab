function chi_1 = wrap(chi_1, chi_2)
    while chi_1 - chi_2 > pi
        chi_1 = chi_1 - 2.0 * pi;
    end
    while chi_1 - chi_2 < -pi
        chi_1 = chi_1 + 2.0 * pi;
    end
end

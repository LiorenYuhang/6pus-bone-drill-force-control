import math
import pytest
from scan import evaluate,SCENARIOS


def test_reference_trajectory_starts_and_returns_to_home():
    results=evaluate(.25,2.,2,.02)
    assert set(results)==set(SCENARIOS)
    for result in results.values():
        rows=result['rows']
        assert len(rows)==201
        for j in range(1,7):
            assert rows[0][f'ik_delta_q{j}_steps']==pytest.approx(0.,abs=1e-8)
            assert rows[-1][f'ik_delta_q{j}_steps']==pytest.approx(0.,abs=1e-7)
        assert all(math.isfinite(x) for x in result['max_ik_vs_shaped_m'])

def test_rejects_invalid_configuration():
    with pytest.raises(ValueError): evaluate(float('nan'),5.,2,.02)
    with pytest.raises(ValueError): evaluate(.25,5.,2,.03)

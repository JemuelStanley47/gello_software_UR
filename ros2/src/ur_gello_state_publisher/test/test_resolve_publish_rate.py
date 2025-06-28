def resolve_publish_rate(user_rate: float, config_rate: float, estimate: float) -> float:
    if user_rate > 0:
        preferred = user_rate
    elif config_rate > 0:
        preferred = config_rate
    else:
        preferred = estimate
    return min(preferred, estimate)


def test_user_preferred_under_hw_limit():
    assert resolve_publish_rate(200.0, 150.0, 500.0) == 200.0


def test_user_preferred_above_hw_limit():
    assert resolve_publish_rate(600.0, 150.0, 500.0) == 500.0


def test_config_preferred_under_hw_limit():
    assert resolve_publish_rate(0.0, 300.0, 500.0) == 300.0


def test_config_preferred_above_hw_limit():
    assert resolve_publish_rate(0.0, 600.0, 500.0) == 500.0


def test_fallback_to_estimate():
    assert resolve_publish_rate(0.0, 0.0, 400.0) == 400.0


def test_all_zero():
    assert resolve_publish_rate(0.0, 0.0, 0.0) == 0.0


def test_user_preferred_zero_but_config_valid():
    assert resolve_publish_rate(0.0, 250.0, 500.0) == 250.0


def test_user_and_config_invalid_but_estimate_valid():
    assert resolve_publish_rate(-1.0, -1.0, 300.0) == 300.0

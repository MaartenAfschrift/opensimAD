"""Isolated CasADi processing of a recorder graph; also usable as a worker."""
import argparse
import importlib.util
from pathlib import Path


def generate(graph_dir, n_inputs, second_order):
    import casadi as ca
    graph_dir = Path(graph_dir).resolve()
    spec = importlib.util.spec_from_file_location('opensimad_recorded_graph', graph_dir / 'foo.py')
    graph = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(graph)
    x = ca.SX.sym('arg', n_inputs)
    y, _, _ = graph.foo(x)
    function = ca.Function('F', [x], [y])
    function.save(str(graph_dir / 'F_foo.casadi'))
    # CodeGenerator requires a basename; the worker's cwd is its private workspace.
    code = ca.CodeGenerator('foo_jac')
    code.add(function)
    code.add(function.jacobian())
    if second_order:
        reverse = function.reverse(1)
        code.add(reverse)
        for i in range(7):
            code.add(function.forward(2**i))
            code.add(reverse.forward(2**i))
    code.generate(str(graph_dir) + '/')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('graph_dir')
    parser.add_argument('n_inputs', type=int)
    parser.add_argument('--second-order', action='store_true')
    args = parser.parse_args()
    generate(args.graph_dir, args.n_inputs, args.second_order)

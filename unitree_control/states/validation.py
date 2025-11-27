from abc import ABCMeta
import ast
import inspect
import textwrap
from typing import Any, Dict


class _LoopCancellationInjector(ast.NodeTransformer):
    def _create_check_node(self, lineno=0, col_offset=0) -> ast.Expr:
        return ast.Expr(
            value=ast.Call(
                func=ast.Attribute(
                    value=ast.Name(id="self", ctx=ast.Load()),
                    attr="check_shutdown",
                    ctx=ast.Load()
                ),
                args=[],
                keywords=[]
            ),
            lineno=lineno,
            col_offset=col_offset
        )

    def visit_While(self, node) -> Any:
        self.generic_visit(node)
        check_node = self._create_check_node(
            lineno=getattr(node, "lineno", 0),
            col_offset=getattr(node, "col_offset", 0)
        )
        node.body.insert(0, check_node)
        return node
    
    def visit_For(self, node) -> Any:
        self.generic_visit(node)
        check_node = self._create_check_node(
            lineno=getattr(node, "lineno", 0),
            col_offset=getattr(node, "col_offset", 0)
        )
        node.body.insert(0, check_node)
        return node


class _CancellableMeta(ABCMeta):
    def __new__(mcls, name, bases, attrs):
        if "execute" in attrs and inspect.isfunction(attrs["execute"]):
            original = attrs["execute"]

            try:
                src = inspect.getsource(original) # get source code 
                src = textwrap.dedent(src) # removing indentation -> needed for parsing with AST
                tree = ast.parse(src) # parse into AST

                injector = _LoopCancellationInjector()
                new_tree = injector.visit(tree)
                ast.fix_missing_locations(new_tree) # fixes line numbers and other metadata for compilation -> cleanup

                env = original.__globals__.copy()
                local_env: Dict[str, Any] = {}

                compiled = compile(new_tree, filename="<ast>", mode="exec") # convert the new AST into executable python code
                exec(compiled, env, local_env) # runs in original context to make sure things like "self" still work

                # if compilation succeeds... the new function replaces the old execute
                new_func = local_env.get(original.__name__)
                if new_func is not None:
                    new_func.__defaults__ = original.__defaults__
                    new_func.__kwdefaults__ = original.__kwdefaults__
                    attrs["execute"] = new_func
                else:
                    attrs["execute"] = original

            except (OSError, IOError, TypeError, IndentationError, SyntaxError, ValueError) as e:
                attrs["execute"] = original # leave as is if exception is raised

        return super().__new__(mcls, name, bases, attrs)
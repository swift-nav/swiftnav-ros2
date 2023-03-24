Changelog
=========

Version 1.0.6 (2022-11-20)
-------------

* Upgrade pyperf from 2.4.1 to 2.5.0
* Add a benchmark to measure gc traversal
* Add jobs field in compile section to specify make -j param
* Add benchmark for Docutils
* Add async_generators benchmark
* Add benchmark for IPC
* Fix Manifest Group
* Fix installing dev build of pyperformance inside compile/compile_all
* Always upload, even when some benchmarks fail
* Add sqlglot benchmarks
* Support reporting geometric mean by tags
* Allow for specifying local wheels and sdists as dependencies
* Add a benchmark based on `python -m pprint`
* Add mdp back into the default group
* Add coroutines benchmark
* Reduce noise in generators benchmark
* Add benchmark for deepcopy
* Add coverage benchmark
* Add generators benchmark
* Add benchmark for async tree workloads
* Support relative paths to manifest files
* Add support for multiple benchmark groups in a manifest
* Fix --inherit-environ issue
* Use working Genshi 0.7.7

Version 1.0.4 (2022-01-25)
-------------

* Re-release support for user-defined benchmark after fixing problem
  with virtual environments.

Version 1.0.3 (2021-12-20)
-------------

* Support user-defined benchmark suites.

Version 1.0.2 (2021-05-11)
-------------

* Disable the henshi benchmark temporarily since is no longer compatible
  with Python 3.11.
* Reenable html5lib benchmark: html5lib 1.1 has been released.
* Update requirements.
* Replace Travis CI with GitHub Actions.
* The development branch ``master`` was renamed to ``main``.
  See https://sfconservancy.org/news/2020/jun/23/gitbranchname/ for the
  rationale.

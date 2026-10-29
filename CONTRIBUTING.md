# Contributing

Any contribution that you make to this repository will
be under the Apache 2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0.html):

~~~
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
~~~

## Guidelines

- Run the tests before a pull request:
  `colcon test --packages-select scene_db --event-handlers console_direct+`
  (unit tests, flake8, pep257, copyright).
- New services or parameters: update `docs/API.md` and, if the workflow
  changes, `docs/TUTORIAL.md`; add a line to `CHANGELOG.rst`.
- Every write to the database goes through one transaction in `store.py`
  and bumps the scene version.

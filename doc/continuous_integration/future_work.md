# Future Work

The CI has room for improvement and may still evolve over time. The following notes alterative integration options, including current pros and cons for each.

##  [GitHub Actions](https://github.com/features/actions)

Github actions is an emerging container based CI service that tightly integrates with the rest of GitHub's service offerings. With a growing ecosystem of official and federated 3rd party actions available, one can compose custom and extensive CI/CD workflows. 

### Pros:

* Self hosted runners
  * Optionally run workflows form on site, not just cloud VMs
  * https://docs.github.com/en/free-pro-team@latest/actions/hosting-your-own-runners
  * Leverage local hardware, e.g: GPUs, persistent storage, robot sensors, etc.

### Cons:

* No test introspection
  * One must still roll there own test result reporting
  * https://github.community/t/publishing-test-results/16215/12
  * Xunit test results are not rendered, aggregated, nor summarized
* Restricted caching
  * Caching with runners is less ergonomic than other CI providers
  * https://github.com/microsoft/azure-pipelines-agent/issues/2043
  * Implementation inherits same limitation from azure-pipelines-agent
* No job level parallelism
  * No equivalent parallelism for splitting tests via timing data
  * https://circleci.com/docs/2.0/parallelism-faster-jobs
  * Parameterizable parallelism without adding jobs to workflow
* No RAM Disk access
  * Useful to improve file IO performance
  * https://circleci.com/docs/2.0/executor-types/#ram-disks
  * Applicable for frequent reads/writes, e.g. ccache
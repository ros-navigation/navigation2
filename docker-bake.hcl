variable "repo" {
  default = "navigation2"
}
variable "registry" {
  default = "ghcr.io"
}

group "default" {
  targets = ["tooler"]
}

################################################################################
// MARK: General targets
################################################################################

target "baser" {
  target = "baser"
  tags = ["nav2:baser"]
  pull = true
  no-cache = false
  cache-from = [
    // "type=registry,ref=${registry}/${repo}:main-tooler.cache",
  ]
  dockerfile = ".docker/Dockerfile"
}

target "cacher" {
  inherits   = ["baser"]
  target = "cacher"
  tags = ["nav2:cacher"]
}

target "runner" {
  inherits   = ["baser"]
  target = "runner"
  tags = ["nav2:runner"]
}

target "prepper" {
  inherits   = ["runner"]
  target = "prepper"
  tags = ["nav2:prepper"]
}

target "validator" {
  inherits   = ["prepper"]
  target = "validator"
  tags = ["nav2:validator"]
}

target "tooler" {
  inherits   = ["validator"]
  target = "tooler"
  tags = ["nav2:tooler"]
}

################################################################################
// MARK: Development targets 
################################################################################

target "dever" {
  inherits   = ["tooler"]
  target = "dever"
  tags = ["nav2:dever"]
}

target "seeder" {
  inherits   = ["dever"]
  target = "seeder"
  tags = ["nav2:seeder"]
  // no-cache-filter = ["builder"]
  args = {
    CLEAR_WS_CACHE = null,
    // CLEAR_WS_CACHE = "${timestamp()}",
    SEED_WS_CACHE = null,
    // SEED_WS_CACHE = "${timestamp()}",
  }
}

target "builder" {
  inherits   = ["seeder"]
  target = "builder"
  tags = ["nav2:builder"]
  // no-cache-filter = ["builder"]
  args = {
    BUST_BUILD_CACHE = null,
    // BUST_BUILD_CACHE = "${timestamp()}",
  }
}

target "tester" {
  inherits   = ["builder"]
  target = "tester"
  tags = ["nav2:tester"]
  args = {
    BUST_TEST_CACHE = null,
    // BUST_TEST_CACHE = "${timestamp()}",
  }
}

target "dancer" {
  inherits   = ["builder"]
  target = "dancer"
  tags = ["nav2:dancer"]
}

target "exporter" {
  inherits   = ["dancer"]
  target = "exporter"
  tags = ["nav2:exporter"]
}

################################################################################
// MARK: Production targets
################################################################################

target "shipper" {
  inherits   = ["dancer"]
  target = "shipper"
  args = {
  }
}

target "releaser" {
  inherits   = ["shipper"]
  target = "releaser"
  tags = ["nav2:releaser"]
  args = {
    SHIP_FROM_STAGE = "runner",
  }
  cache-from = [
    // "type=registry,ref=${registry}/${repo}:main-releaser.cache",
  ]
}

target "debugger" {
  inherits   = ["shipper"]
  target = "debugger"
  tags = ["nav2:debugger"]
  args = {
    SHIP_FROM_STAGE = "tooler",
  }
  cache-from = [
    // "type=registry,ref=${registry}/${repo}:main-debugger.cache",
  ]
}
